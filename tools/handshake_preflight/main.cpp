// Read-only model checks. No Unitree SDK, authority client, command publishers,
// controller manager or trajectory execution APIs are linked or constructed.
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <brsd_upbody/common/arm_ik_seeds.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <set>

std::string read(const std::string& path) {
  std::ifstream file(path); if (!file) throw std::runtime_error("Cannot read "+path);
  return std::string(std::istreambuf_iterator<char>(file), {});
}
YAML::Node vector_node(const std::vector<double>& values) {
  YAML::Node n; for (double v:values)n.push_back(v);return n;
}
int main(int argc,char** argv) {
  if(argc!=7 && argc!=8){std::cerr<<"usage: preflight URDF SRDF LIMITS INPUT_JSONL OUTPUT_YAML MAX_SAMPLES [STATE_YAML]\n";return 2;}
  try {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("handshake_model_preflight");
    for(const auto& arm:{"left_arm","right_arm"}) {
      for(const auto& prefix:{std::string(arm),"robot_description_kinematics."+std::string(arm)}) {
        node->declare_parameter(prefix+".kinematics_solver",std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
        node->declare_parameter(prefix+".kinematics_solver_timeout",.01);
      }
    }
    const auto limits=YAML::LoadFile(argv[3]);
    for(const auto& entry:limits["joint_limits"]) {
      const auto prefix="robot_description_planning.joint_limits."+entry.first.as<std::string>()+".";
      for(const auto& key:{"has_velocity_limits","has_acceleration_limits"})
        if(entry.second[key])node->declare_parameter(prefix+key,entry.second[key].as<bool>());
      for(const auto& key:{"max_velocity","max_acceleration"})
        if(entry.second[key])node->declare_parameter(prefix+key,entry.second[key].as<double>());
    }
    for(const auto& entry:limits["cartesian_limits"])
      node->declare_parameter("robot_description_planning.cartesian_limits."+entry.first.as<std::string>(),entry.second.as<double>());
    robot_model_loader::RobotModelLoader::Options options(read(argv[1]),read(argv[2]));
    options.load_kinematics_solvers_=true;
    robot_model_loader::RobotModelLoader loader(node,options);
    auto model=loader.getModel();if(!model)throw std::runtime_error("Cannot load robot model");
    for(const auto* link:model->getLinkModels())
      if(model->getURDF()->getLink(link->getName())->collision && link->getShapes().empty())
        throw std::runtime_error("Missing collision geometry: "+link->getName());
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);
    auto pipeline=std::make_shared<planning_pipeline::PlanningPipeline>(model,node,"",
      "pilz_industrial_motion_planner/CommandPlanner",std::vector<std::string>{});
    pipeline->displayComputedMotionPlans(false);pipeline->publishReceivedRequests(false);
    moveit::core::RobotState initial(model);initial.setToDefaultValues();
    initial.setJointGroupPositions("left_arm",std::vector<double>{.2844,.1302,.0099,.9844,.0613,.0845,-.0472});
    initial.setJointGroupPositions("right_arm",std::vector<double>{.2904,-.1219,.0129,.9785,-.0545,-.0249,-.0351});
    std::string start_source="recorded_neutral_fixture_not_current_robot";
    bool full_body=false;
    if(argc==8) {
      auto captured=YAML::LoadFile(argv[7]);
      if(!captured["sdk_connected"].as<bool>())throw std::runtime_error("State snapshot has no real SDK connection");
      const auto positions=captured["joint_positions"];
      std::vector<std::string> required;
      for(const auto& name:{"left_arm","right_arm"}) {
        const auto& names=model->getJointModelGroup(name)->getVariableNames();
        required.insert(required.end(),names.begin(),names.end());
      }
      for(const auto& name:{"waist_yaw_joint","waist_roll_joint","waist_pitch_joint"})required.push_back(name);
      full_body=captured["full_body_state"] && captured["full_body_state"].as<bool>();
      if(full_body)required=model->getVariableNames();
      for(const auto& name:required) {
        if(!positions[name])throw std::runtime_error("State snapshot missing "+name);
        double value=positions[name].as<double>();
        if(!std::isfinite(value))throw std::runtime_error("Nonfinite measured joint");
        initial.setVariablePosition(name,value);
      }
      start_source=full_body?"real_29_joint_snapshot":"real_upper_body_snapshot_lower_body_nominal";
    }
    initial.update();
    if(!initial.satisfiesBounds() || scene->isStateColliding(initial,"",false)) {
      for(const auto& name:model->getVariableNames()) {
        const auto& bound=model->getVariableBounds(name);double value=initial.getVariablePosition(name);
        if(bound.position_bounded_ && (value<bound.min_position_ || value>bound.max_position_))
          std::cerr<<"INITIAL_LIMIT "<<name<<" q="<<value<<" min="<<bound.min_position_<<" max="<<bound.max_position_<<std::endl;
      }
      collision_detection::CollisionRequest check;check.contacts=true;check.max_contacts=30;
      collision_detection::CollisionResult collisions;scene->checkSelfCollision(check,collisions,initial);
      for(const auto& pair:collisions.contacts)
        std::cerr<<"INITIAL_COLLISION "<<pair.first.first<<" <-> "<<pair.first.second<<std::endl;
      throw std::runtime_error("Initial state is outside limits or self-colliding");
    }
    // Production uses the driver's common planner. This mode checks only the
    // measured full-body start; it must not veto a driver's bounded detour
    // with this diagnostic's separate PTP-only target solver.
    if (std::string(argv[6]) == "0") {
      if (argc != 8 || !full_body)
        throw std::runtime_error("State-only mode requires a real full-body snapshot");
      YAML::Node report;
      report["start_state_valid"] = true;
      report["start_state"] = start_source;
      report["hardware_commands_sent"] = false;
      report["target_planning"] = "driver_common_arm_transit";
      std::ofstream output(argv[5]); output << report << "\n";
      rclcpp::shutdown(); return 0;
    }
    std::ifstream input(argv[4]);if(!input)throw std::runtime_error("No input goals");
    std::vector<YAML::Node> goals;std::string line;
    while(std::getline(input,line))if(!line.empty())goals.push_back(YAML::Load(line));
    int sample_count=std::min(static_cast<int>(goals.size()),std::stoi(argv[6]));
    if(sample_count<1)throw std::runtime_error("No goals to check");
    YAML::Node report,records;
    report["source_targets"]=goals.size();report["checked_targets"]=sample_count;
    report["start_state"]=start_source;
    report["lower_body_state_nominal"]=!full_body;
    report["real_world_environment_checked"]=false;report["hardware_commands_sent"]=false;
    report["planner"]=std::string("PilzPTP_fixed_joint_goal_no_OMPL");
    int accepted=0;
    for(int sample=0;sample<sample_count;++sample) {
      size_t index=sample_count==1?0:sample*(goals.size()-1)/(sample_count-1);
      auto request=goals[index];const auto arm=request["arm"].as<std::string>();
      if(arm!="left" && arm!="right")throw std::runtime_error("Bad arm");
      const auto* group=model->getJointModelGroup(arm+"_arm");
      if(!group || !group->getSolverInstance())throw std::runtime_error("Missing arm IK solver");
      const auto eef=arm+"_wrist_yaw_link";
      auto xyz=request["wrist_xyz_m"].as<std::vector<double>>();auto q=request["wrist_quaternion_xyzw"].as<std::vector<double>>();
      if(xyz.size()!=3 || q.size()!=4)throw std::runtime_error("Bad target dimensions");
      Eigen::Isometry3d target=Eigen::Isometry3d::Identity();
      target.translation()=Eigen::Vector3d(xyz[0],xyz[1],xyz[2]);
      target.linear()=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).normalized().toRotationMatrix();
      target=initial.getGlobalLinkTransform("torso_link")*target;
      std::vector<double> neutral;initial.copyJointGroupPositions(group,neutral);
      auto batches=brsd::upbody::g1::BuildArmIkSeedBatches(group->getVariableNames(),neutral);
      auto seeds=batches.local;seeds.insert(seeds.end(),batches.coupled.begin(),batches.coupled.end());
      YAML::Node record;record["source_index"]=index;record["arm"]=arm;record["stamp_s"]=request["stamp_s"];
      bool found=false,any_ik=false,any_bounds=false,any_fk=false;int attempted=0;
      std::set<std::string> collision_pairs;
      moveit::core::RobotState goal(initial);
      for(const auto& seed:seeds) {
        ++attempted;moveit::core::RobotState candidate(initial);
        candidate.setJointGroupPositions(group,seed);candidate.enforceBounds(group);candidate.update();
        if(!candidate.setFromIK(group,target,eef,.01))continue;
        any_ik=true;candidate.update();
        if(!candidate.satisfiesBounds())continue;any_bounds=true;
        auto actual=candidate.getGlobalLinkTransform(eef);
        if((actual.translation()-target.translation()).norm()>.002 || Eigen::AngleAxisd(actual.linear().transpose()*target.linear()).angle()>.01)continue;
        any_fk=true;
        collision_detection::CollisionRequest check;check.contacts=true;check.max_contacts=20;
        collision_detection::CollisionResult collision;
        scene->checkSelfCollision(check,collision,candidate);
        if(collision.collision) {
          for(const auto& pair:collision.contacts)if(collision_pairs.size()<12)
            collision_pairs.insert(pair.first.first+" <-> "+pair.first.second);
          continue;
        }
        goal=candidate;found=true;break;
      }
      record["seed_attempts"]=attempted;record["ik_found"]=any_ik;
      record["bounded_solution_found"]=any_bounds;record["collision_free_ik_found"]=found;
      record["accurate_fk_solution_found"]=any_fk;
      record["precontact_palm_xyz_m"]=request["precontact_palm_xyz_m"];
      record["wrist_xyz_m"]=request["wrist_xyz_m"];
      for(const auto& pair:collision_pairs)record["collision_pairs"].push_back(pair);
      record["ptp_path_valid"]=false;
      if(found) {
        std::vector<double> joints;goal.copyJointGroupPositions(group,joints);record["goal_joints_rad"]=vector_node(joints);
        planning_interface::MotionPlanRequest plan;plan.group_name=arm+"_arm";plan.planner_id="PTP";
        plan.allowed_planning_time=2.;plan.max_velocity_scaling_factor=.15;plan.max_acceleration_scaling_factor=.10;
        moveit::core::robotStateToRobotStateMsg(initial,plan.start_state);
        plan.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal,group));
        planning_interface::MotionPlanResponse response;
        bool planned=pipeline->generatePlan(scene,plan,response) && response.error_code_.val==moveit_msgs::msg::MoveItErrorCodes::SUCCESS && response.trajectory_ && response.trajectory_->getWayPointCount()>0;
        bool valid=planned;size_t checked=0;
        if(planned) {
          auto previous=initial;
          for(size_t i=0;i<response.trajectory_->getWayPointCount() && valid;++i) {
            const auto& next=response.trajectory_->getWayPoint(i);
            std::vector<double> a,b;previous.copyJointGroupPositions(group,a);next.copyJointGroupPositions(group,b);
            double span=0.;for(size_t j=0;j<a.size();++j)span=std::max(span,std::abs(a[j]-b[j]));
            int steps=std::max(1,static_cast<int>(std::ceil(span/.02)));
            for(int step=1;step<=steps && valid;++step) {
              moveit::core::RobotState state(model);previous.interpolate(next,static_cast<double>(step)/steps,state);state.update();
              ++checked;valid=state.satisfiesBounds() && !scene->isStateColliding(state,"",false);
            }
            previous=next;
          }
        }
        record["planning_error_code"]=response.error_code_.val;record["path_states_checked"]=checked;
        record["ptp_path_valid"]=valid;
        record["reason"]=valid?"fixed_goal_PTP_passed":planned?"PTP_path_collision_or_limits":"PTP_planning_failed";
        if(valid)++accepted;
      } else record["reason"]=!any_ik?"no_IK_found_in_fixed_seed_search":!any_bounds?"joint_limits":!any_fk?"IK_FK_tolerance_failed":"self_collision";
      records.push_back(record);
      std::cout<<"CHECK "<<sample+1<<"/"<<sample_count<<" "<<arm<<" "<<record["reason"].as<std::string>()<<std::endl;
    }
    report["accepted_targets"]=accepted;report["results"]=records;
    std::ofstream output(argv[5]);output<<report<<"\n";
    rclcpp::shutdown();return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
