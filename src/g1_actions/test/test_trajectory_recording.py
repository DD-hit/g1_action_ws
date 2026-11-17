import ast,copy,tempfile,unittest
from pathlib import Path
from g1_actions.actions.elevator.trajectory_recording import ApproachCapture,select_record,save_json


class CaptureTests(unittest.TestCase):
    def fixture(self):
        c=ApproachCapture();c.begin({'stage':'APPROACH','arm':'right','direction':'up'},10.)
        return c

    def sample(self,c,t=10.):c.sample(t,[.1]*7,[.2]*7,[.3]*3,.01,1)

    def test_left_arm_and_other_phases_do_not_begin(self):
        c=ApproachCapture()
        self.assertFalse(c.begin({'stage':'APPROACH','arm':'left'},0.))
        self.assertFalse(c.begin({'stage':'PRESS','arm':'right'},0.))
        self.assertIsNone(c.active)

    def test_complete_capture_preserves_both_arms_and_waist(self):
        c=self.fixture();self.sample(c);self.sample(c,10.1)
        r=c.finish('reached',True)
        self.assertTrue(r['capture_valid']);self.assertFalse(r['path_safety_verified'])
        self.assertEqual(r['samples'][0]['left_q_rad'],[.2]*7)
        self.assertEqual(r['samples'][0]['waist_q_rad'],[.3]*3)

    def test_stale_missing_or_faulty_feedback_is_not_selectable(self):
        for right,age,fault in (([float('nan')]*7,.01,False),([0.]*6,.01,False),([0.]*7,.7,False),([0.]*7,.01,True)):
            c=self.fixture();c.sample(10.,right,[0.]*7,[0.]*3,age,1,fault);self.sample(c,10.1)
            r=c.finish('reached',True)
            self.assertFalse(r['capture_valid'])
            with self.assertRaises(ValueError):select_record(r)

    def test_feedback_gap_rejects_capture(self):
        c=self.fixture();self.sample(c);self.sample(c,10.3)
        self.assertFalse(c.finish('reached',True)['capture_valid'])

    def test_aborted_or_interrupted_capture_is_not_selectable(self):
        c=self.fixture();self.sample(c);self.sample(c,10.1)
        r=c.finish('interrupted')
        with self.assertRaises(ValueError):select_record(r)

    def test_prefix_selection_preserves_start_and_original(self):
        c=self.fixture()
        for t in (10.,10.1,10.2,10.3):self.sample(c,t)
        r=c.finish('reached',True);original=copy.deepcopy(r)
        chosen=select_record(r,.15)
        self.assertEqual(len(chosen['samples']),2)
        self.assertEqual(chosen['samples'][0],r['samples'][0]);self.assertEqual(r,original)
        self.assertFalse(chosen['path_safety_verified'])
        for end in (0.,100.,float('nan')):
            with self.assertRaises(ValueError):select_record(r,end)

    def test_save_is_valid_json(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'trial.json';save_json(p,{'arm':'right'})
            self.assertEqual(json.loads(p.read_text()),{'arm':'right'})

    def test_recorder_has_no_publish_or_service_call_api(self):
        source=Path(__file__).resolve().parents[1]/'scripts/record_right.py'
        forbidden={'create_publisher','create_client','call_async','publish','trigger','move','move_joints'}
        tree=ast.parse(source.read_text())
        self.assertFalse([n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in forbidden])


if __name__=='__main__':unittest.main()
