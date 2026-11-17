"""Metric palm surface pose in the RGB optical frame, from aligned depth."""
import math
import numpy as np

PALM = [0, 5, 9, 13, 17]


def validate_pair(color, depth, info, now_s, config):
    """Metadata dictionaries keep synchronization checks independent of ROS."""
    skew = abs(color['stamp_s'] - depth['stamp_s'])
    if skew > config.get('max_sync_skew_s', .04):
        raise ValueError('RGB/depth timestamp mismatch')
    age = now_s - color['stamp_s']
    if age < -.05 or age > config.get('max_frame_age_s', .6):
        raise ValueError('RGB-D is stale or clock is inconsistent')
    if not color['frame_id'] or len({color['frame_id'], depth['frame_id'], info['frame_id']}) != 1:
        raise ValueError('RGB, aligned depth and CameraInfo must share an optical frame')
    sizes = {(item['width'], item['height']) for item in (color, depth, info)}
    if len(sizes) != 1:
        raise ValueError('RGB/depth/intrinsic image dimensions differ')
    return skew


def image_array(message):
    """Decode ROS Image layout without cv_bridge or copying through NumPy ABI."""
    encoding = message.encoding
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if encoding in channels:
        count = channels[encoding]
        dtype, shape, strides = np.dtype('uint8'), (message.height, message.width, count), (message.step, count, 1)
        minimum_step = message.width * count
    elif encoding in ('16UC1', 'mono16', '32FC1'):
        dtype = np.dtype(('>' if message.is_bigendian else '<') + ('f4' if encoding == '32FC1' else 'u2'))
        shape, strides = (message.height, message.width), (message.step, dtype.itemsize)
        minimum_step = message.width * dtype.itemsize
    else:
        raise ValueError('Unsupported image encoding: ' + encoding)
    if message.step < minimum_step or len(message.data) != message.step * message.height:
        raise ValueError('Invalid image stride or buffer size')
    result = np.ndarray(shape, dtype=dtype, buffer=memoryview(message.data), strides=strides)
    if encoding in ('16UC1', 'mono16'):
        return result.astype(np.float32) * .001
    return result.copy()


def rays(pixels, intrinsics):
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    fx, fy, cx, cy = (float(intrinsics[k]) for k in ('fx', 'fy', 'cx', 'cy'))
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise ValueError('Invalid camera intrinsics')
    coefficients = np.asarray(intrinsics.get('d', []), dtype=float)
    if not np.isfinite(coefficients).all():
        raise ValueError('Invalid distortion coefficients')
    if np.any(np.abs(coefficients) > 1e-12):
        import cv2
        camera = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        model = intrinsics.get('distortion_model', '')
        if model == 'equidistant':
            xy = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), camera, coefficients)
        elif model in ('plumb_bob', 'rational_polynomial'):
            xy = cv2.undistortPoints(pixels.reshape(-1, 1, 2), camera, coefficients)
        else:
            raise ValueError('Unsupported distortion model: ' + model)
        xy = xy.reshape(-1, 2)
    else:
        xy = (pixels - [cx, cy]) / [fx, fy]
    return np.column_stack((xy, np.ones(len(xy))))


def quaternion_xyzw(rotation):
    """Stable conversion, including rotations close to pi."""
    r = np.asarray(rotation, dtype=float)
    q = np.empty(4)
    trace = np.trace(r)
    if trace > 0:
        scale = math.sqrt(trace + 1.) * 2
        q[:] = [(r[2, 1]-r[1, 2])/scale, (r[0, 2]-r[2, 0])/scale,
                (r[1, 0]-r[0, 1])/scale, scale/4]
    else:
        i = int(np.argmax(np.diag(r))); j = (i+1) % 3; k = (i+2) % 3
        scale = math.sqrt(max(0., 1.+r[i, i]-r[j, j]-r[k, k])) * 2
        q[i] = scale/4
        q[j] = (r[j, i]+r[i, j])/scale
        q[k] = (r[k, i]+r[i, k])/scale
        q[3] = (r[k, j]-r[j, k])/scale
    return q / np.linalg.norm(q)


def estimate_palm_pose(landmarks, aligned_depth, intrinsics, *, handedness, config=None):
    """Plane-fit palm pixels and intersect rays; model-relative z is unused."""
    import cv2
    config = config or {}
    try:
        if handedness not in ('Right', 'Left'):
            raise ValueError('Handedness is not confirmed')
        landmarks = np.asarray(landmarks, dtype=float)
        if landmarks.shape != (21, 3) or not np.isfinite(landmarks).all():
            raise ValueError('Invalid landmarks')
        depth = np.asarray(aligned_depth)
        if depth.ndim != 2 or depth.shape != (intrinsics['height'], intrinsics['width']):
            raise ValueError('Aligned depth dimensions do not match intrinsics')
        h, w = depth.shape
        pixels = landmarks[:, :2] * [w, h]
        if np.any(pixels < 0) or np.any(pixels > [w-1, h-1]):
            raise ValueError('Hand landmarks are clipped by image boundary')
        palm = pixels[PALM]
        if np.any(palm < 0) or np.any(palm > [w-1, h-1]):
            raise ValueError('Palm is clipped by image boundary')
        center = (pixels[0]+pixels[[5, 9, 13, 17]].mean(axis=0))/2
        hull = cv2.convexHull(np.round(palm).astype(np.int32))
        if cv2.contourArea(hull) < config.get('min_palm_area_px', 100):
            raise ValueError('Palm image region is too small or edge-on')
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(mask, hull, 1)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
        low, high = config.get('min_depth_m', .2), config.get('max_depth_m', 1.2)
        valid = np.isfinite(depth) & (depth >= low) & (depth <= high)
        u, v = np.round(center).astype(int)
        patch = depth[max(0, v-4):min(h, v+5), max(0, u-4):min(w, u+5)]
        patch = patch[np.isfinite(patch) & (patch >= low) & (patch <= high)]
        if len(patch) < 5:
            raise ValueError('No reliable depth near palm center')
        center_depth = float(np.median(patch))
        ys, xs = np.nonzero(mask.astype(bool) & valid & (np.abs(depth-center_depth) < .10))
        minimum = config.get('min_depth_points', 80)
        if len(xs) < minimum:
            raise ValueError('Insufficient valid palm depth samples')
        if len(xs) > 800:
            indices = np.linspace(0, len(xs)-1, 800).astype(int)
            xs, ys = xs[indices], ys[indices]
        cloud = rays(np.column_stack((xs, ys)), intrinsics) * depth[ys, xs, None]
        threshold = config.get('plane_inlier_m', .004)
        rng = np.random.default_rng(17)
        best = np.zeros(len(cloud), dtype=bool)
        for _ in range(60):
            a, b, c = cloud[rng.choice(len(cloud), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            length = np.linalg.norm(normal)
            if length < 1e-9:
                continue
            normal /= length
            inliers = np.abs((cloud-a)@normal) <= threshold
            if inliers.sum() > best.sum():
                best = inliers
        if best.sum() < max(minimum, len(cloud)*config.get('min_inlier_fraction', .6)):
            raise ValueError('No consistent palm depth plane')
        points = cloud[best]
        centroid = points.mean(axis=0)
        _, singular, vt = np.linalg.svd(points-centroid, full_matrices=False)
        if singular[1] < 1e-5:
            raise ValueError('Degenerate palm depth plane')
        normal = vt[-1]
        residual = float(np.sqrt(np.mean(((points-centroid)@normal)**2)))
        if residual > config.get('max_plane_residual_m', .004):
            raise ValueError('Palm plane residual is excessive')
        directions = rays(np.vstack((center, palm)), intrinsics)
        denominators = directions@normal
        if np.any(np.abs(denominators) < .15):
            raise ValueError('Palm plane is edge-on to camera')
        distances = float(normal@centroid)/denominators
        if not np.isfinite(distances).all() or np.any(distances < low) or np.any(distances > high):
            raise ValueError('Invalid palm ray/plane intersection')
        metric = directions * distances[:, None]
        origin, wrist, index, middle, ring, pinky = metric
        if abs(origin[2]-center_depth) > config.get('max_center_plane_difference_m', .015):
            raise ValueError('Center depth disagrees with fitted palm plane')
        span = float(np.linalg.norm(index-pinky))
        forward = (index+middle+ring+pinky)/4-wrist
        if not .025 <= span <= .14 or not .025 <= np.linalg.norm(forward) <= .16:
            raise ValueError(f'Metric palm dimensions are implausible: width={span:.4f} m, length={np.linalg.norm(forward):.4f} m')
        hint = np.cross(index-pinky, forward) * (1 if handedness == 'Right' else -1)
        if normal@hint < 0:
            normal = -normal
        y = forward-normal*(forward@normal); y /= np.linalg.norm(y)
        x = np.cross(y, normal); x /= np.linalg.norm(x)
        y = np.cross(normal, x)
        rotation = np.column_stack((x, y, normal))
        return dict(valid=True, position_m=origin.tolist(), rotation=rotation.tolist(),
                    quaternion_xyzw=quaternion_xyzw(rotation).tolist(),
                    valid_depth_points=int(best.sum()), plane_residual_m=residual,
                    inlier_fraction=float(best.mean()), palm_width_m=span,
                    covariance_valid=False, reason='measured palm plane')
    except (ValueError, np.linalg.LinAlgError) as error:
        return dict(valid=False, reason=str(error), covariance_valid=False)


def estimate_palm_position(landmarks, aligned_depth, intrinsics, *, config=None):
    """Measured wrist + fixed distance toward measured MCPs; no orientation."""
    config = config or {}
    try:
        points = np.asarray(landmarks, dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points[:, :2]).all():
            raise ValueError('Invalid image landmarks')
        depth = np.asarray(aligned_depth)
        h, w = intrinsics['height'], intrinsics['width']
        if depth.shape != (h, w):
            raise ValueError('Aligned depth dimensions do not match intrinsics')
        low, high = config.get('min_depth_m', .2), config.get('max_depth_m', 1.2)
        options = config.get('wrist_offset', {})
        offset = float(options.get('distance_m', .06))
        radius = int(options.get('patch_radius_px', 3))
        minimum = int(options.get('min_patch_samples', 8))
        required = int(options.get('min_finger_bases', 3))
        if not 0 < offset <= .15 or not 1 <= radius <= 10 or not 1 <= minimum <= (2*radius+1)**2 or not 2 <= required <= 4:
            raise ValueError('Invalid wrist-offset configuration')

        def sample(index):
            pixel = points[index, :2]*[w, h]
            if np.any(pixel < [radius, radius]) or np.any(pixel > [w-1-radius, h-1-radius]):
                raise ValueError(f'Landmark {index} depth patch is clipped')
            u, v = np.round(pixel).astype(int)
            patch = depth[v-radius:v+radius+1, u-radius:u+radius+1]
            values = patch[np.isfinite(patch) & (patch >= low) & (patch <= high)]
            if len(values) < minimum:
                raise ValueError(f'Landmark {index} has insufficient depth')
            median = float(np.median(values))
            mad = float(np.median(np.abs(values-median)))
            if mad > options.get('max_depth_mad_m', .012):
                raise ValueError(f'Landmark {index} depth is discontinuous')
            return rays([pixel], intrinsics)[0]*median, len(values)

        wrist, count = sample(0)
        bases, accepted = [], []
        for index in (5, 9, 13, 17):
            try:
                base, samples = sample(index)
            except ValueError:
                continue
            distance = np.linalg.norm(base-wrist)
            if .02 <= distance <= options.get('max_wrist_to_base_m', .16):
                bases.append(base); accepted.append(index); count += samples
        if len(bases) < required:
            raise ValueError('Insufficient reliable finger-base depths for 3D direction')
        center = np.mean(bases, axis=0)
        forward = center-wrist
        length = float(np.linalg.norm(forward))
        if length < .02:
            raise ValueError('Wrist-to-finger direction is degenerate')
        direction = forward/length
        position = wrist+offset*direction
        if not low <= position[2] <= high:
            raise ValueError('Estimated palm position is outside depth range')
        return dict(valid=True, position_m=position.tolist(), wrist_position_m=wrist.tolist(),
                    finger_base_center_m=center.tolist(), direction=direction.tolist(),
                    offset_m=offset, accepted_finger_bases=accepted,
                    valid_depth_points=int(count), position_only=True, orientation_valid=False,
                    covariance_valid=False, reason='measured wrist plus fixed offset')
    except (ValueError, TypeError, np.linalg.LinAlgError) as error:
        return dict(valid=False, position_only=True, orientation_valid=False,
                    covariance_valid=False, reason=str(error))
