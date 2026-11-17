#include "distributedMapping.h"
#include <cmath>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: handle message callback 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::globalDescriptorHandler(
	const dcl_slam::global_descriptorConstPtr& msg,
	int& id)
{
	// save timestamp
	robots[id].time_cloud_input_stamp = msg->header.stamp;
	robots[id].time_cloud_input = robots[id].time_cloud_input_stamp.toSec();

	// save descriptors
	LOG(INFO) << "[globalDescriptorHandler(" << id << ")]" <<
		" saveDescriptorAndKey:" << msg->index <<  "." << endl;
	// keyframe_descriptor->saveDescriptorAndKey(msg->values.data(), id, msg->index);
	store_descriptors.emplace_back(make_pair(id,*msg));
}

void distributedMapping::optStateHandler(
	const dcl_slam::optimization_stateConstPtr& msg, int& id)
{
	if(!dcl_slam::OptimizationRound::valid(*msg, number_of_robots_) ||
		!dcl_slam::OptimizationRound::contains(msg->participants, id_) ||
		!dcl_slam::OptimizationRound::contains(msg->participants, id))
		return;
	if(msg->round_id > round.status.round_id && id == msg->coordinator &&
		msg->state <= int(OptimizerState::Initialization) && !msg->committed && !msg->aborted)
	{
		// A lower-ID coordinator may supersede a round when components merge.
		const auto& members = connected_members;
		if(!members.empty() && msg->coordinator > members.front()) return;
		if(optimizer_state != OptimizerState::Idle && optimizer_state < OptimizerState::End) abortOptimization(false);
		beginRound(*msg);
	}
	if(!round.same(*msg)) return;
	if(round.status.committed && msg->committed && !msg->aborted && !round.matchingCommit(*msg))
	{
		ROS_ERROR("Conflicting commit from %d in round %llu", id, (unsigned long long)msg->round_id);
		abortOptimization(true);
		return;
	}
	auto previous = round.peers.find(id);
	bool progress = previous == round.peers.end() ||
		previous->second->state != msg->state || previous->second->committed != msg->committed ||
		previous->second->rotation_done != msg->rotation_done ||
		previous->second->pose_done != msg->pose_done;
	if(round.receive(id, msg) && progress) round_progress = ros::WallTime::now();
}

void distributedMapping::neighborRotationHandler(
	const dcl_slam::neighbor_estimateConstPtr& msg, int& id)
{
	receiveEstimate(msg, id, true);
}

void distributedMapping::neighborPoseHandler(
	const dcl_slam::neighbor_estimateConstPtr& msg, int& id)
{
	receiveEstimate(msg, id, false);
}

void distributedMapping::receiveEstimate(
	const dcl_slam::neighbor_estimateConstPtr& msg, int id, bool rotation)
{
	if(!round.acceptsEstimate(id_,id,*msg)) return;
	const size_t width = rotation ? 9 : 6;
	if(msg->estimate.size() != width * msg->pose_id.size() ||
		(!rotation && msg->anchor_offset.size() != 3) ||
		msg->iteration > uint32_t(optmization_maximum_iteration_) ||
		(msg->estimation_done && !msg->iteration) ||
		!std::all_of(msg->estimate.begin(), msg->estimate.end(), [](float x){return std::isfinite(x);}) ||
		!std::all_of(msg->anchor_offset.begin(), msg->anchor_offset.end(), [](float x){return std::isfinite(x);}))
	{
		ROS_ERROR("Invalid estimate from %d in round %llu", id, (unsigned long long)msg->round_id);
		abortOptimization(true);
		return;
	}
	auto& inbox = rotation ? rotation_inbox : pose_inbox;
	if(round.retainEstimate(id_,id,msg,inbox)) round_progress = ros::WallTime::now();
}

void distributedMapping::publishEstimates(bool rotation, bool rebuild)
{
	auto& outbox = rotation ? rotation_outbox : pose_outbox;
	if(rebuild)
	{
		for(int other : round.status.participants)
		{
			if(other == id_) continue;
			auto& msg = outbox[other];
			msg.header.stamp = ros::Time::now();
			msg.round_id = round.status.round_id;
			msg.receiver_id = other;
			msg.iteration = rotation ? current_rotation_estimate_iteration : current_pose_estimate_iteration;
			msg.initialized = msg.iteration != 0;
			msg.local_converged = rotation ? rotation_local_converged : pose_local_converged;
			msg.estimation_done = rotation ? round.status.rotation_done : round.status.pose_done;
			msg.pose_id.clear();
			msg.estimate.clear();
			msg.anchor_offset.clear();
			for(Key key : round_send_keys[other])
			{
				msg.pose_id.push_back(Symbol(key).index());
				Vector estimate = rotation ? optimizer->linearizedRotationAt(key) : optimizer->linearizedPoses().at(key);
				for(int j = 0; j < estimate.size(); ++j) msg.estimate.push_back(estimate[j]);
			}
			if(!rotation)
				for(int j = 0; j < 3; ++j) msg.anchor_offset.push_back(anchor_offset[j]);
		}
	}
	for(int other : round.status.participants)
		if(other != id_)
		{
			if(rotation) robots[id_].pub_neighbor_rotation_estimates.publish(outbox[other]);
			else robots[id_].pub_neighbor_pose_estimates.publish(outbox[other]);
		}
}

bool distributedMapping::applyNeighborEstimates(bool rotation, bool final_only)
{
	auto& inbox = rotation ? rotation_inbox : pose_inbox;
	if(final_only && !round.finalEstimatesReady(id_,inbox)) return false;
	optimizer->clearNeighboringRobotInit();
	for(int other : round.status.participants)
	{
		if(other == id_) continue;
		const auto& msg = *inbox.at(other);
		const auto& expected = round_receive_keys[other];
		if(msg.pose_id.size() != expected.size())
			throw std::runtime_error("Committed separator payload size mismatch");
		const size_t width = rotation ? 9 : 6;
		for(size_t i = 0; i < expected.size(); ++i)
		{
			if(msg.pose_id[i] != int32_t(Symbol(expected[i]).index()))
				throw std::runtime_error("Committed separator key mismatch");
			Vector estimate(width);
			for(size_t j = 0; j < width; ++j) estimate[j] = msg.estimate[width*i+j];
			if(rotation) optimizer->updateNeighborLinearizedRotations(expected[i], estimate);
			else optimizer->updateNeighborLinearizedPoses(expected[i], estimate);
		}
		optimizer->updateNeighboringRobotInitialized(char('a'+other), msg.initialized);
		if(!rotation && msg.estimation_done)
			neighbors_anchor_offset[other] = Point3(msg.anchor_offset[0], msg.anchor_offset[1], msg.anchor_offset[2]);
	}
	return true;
}

void distributedMapping::estimateStep(bool rotation)
{
	auto& inbox = rotation ? rotation_inbox : pose_inbox;
	int& iteration = rotation ? current_rotation_estimate_iteration : current_pose_estimate_iteration;
	auto& done = rotation ? round.status.rotation_done : round.status.pose_done;
	auto& local_converged = rotation ? rotation_local_converged : pose_local_converged;
	if(done) return;
	if(iteration > 0 && round.allConvergedAt(id_, uint32_t(iteration), local_converged,
		uint32_t(optmization_maximum_iteration_), inbox))
	{
		done = true;
		round_progress = ros::WallTime::now();
		publishEstimates(rotation, true);
		publishRoundState();
		return;
	}
	if(!round.stepReady(id_, iteration, inbox)) return;
	applyNeighborEstimates(rotation,false);
	if(rotation) { optimizer->estimateRotation(); optimizer->updateRotation(); }
	else { optimizer->estimatePoses(); optimizer->updatePoses(); }
	optimizer->updateInitialized(true);
	++iteration;
	if(!std::isfinite(optimizer->latestChange()))
		throw std::runtime_error("Nonfinite distributed estimate change");
	const double threshold = rotation ? rotation_estimate_change_threshold_ : pose_estimate_change_threshold_;
	const int min_sweeps = std::min(2, optmization_maximum_iteration_);
	local_converged = iteration >= min_sweeps && optimizer->latestChange() <= threshold;
	const bool global_converged = round.allConvergedAt(id_, uint32_t(iteration), local_converged,
		uint32_t(optmization_maximum_iteration_), inbox);
	done = global_converged || iteration >= optmization_maximum_iteration_;
	if(done)
		ROS_INFO("Round %llu robot %d %s finished at iter %d (reason=%s, latestChange=%.3e)",
			(unsigned long long)round.status.round_id, id_, rotation ? "rotation" : "pose",
			iteration, global_converged ? "converged" : "max_iter", optimizer->latestChange());
	if(!rotation)
	{
		Key first = optimizer->currentEstimate().keys().front();
		anchor_point = initial_values->at<Pose3>(first).translation();
		anchor_offset = anchor_point - (optimizer->currentEstimate().at<Pose3>(first).translation() +
			Point3(optimizer->linearizedPoses().at(first).tail(3)));
	}
	round_progress = ros::WallTime::now();
	publishEstimates(rotation, true);
	publishRoundState();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: saving keyframe API
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::performDistributedMapping(
	const Pose3& pose_to,
	const pcl::PointCloud<PointPose3D>::Ptr frame_to,
	const ros::Time& timestamp)
{
	// save keyframe cloud
	pcl::copyPointCloud(*frame_to, *robots[id_].keyframe_cloud);
	robots[id_].keyframe_cloud_array.push_back(*robots[id_].keyframe_cloud);
	// save timestamp
	robots[id_].time_cloud_input_stamp = timestamp;
	robots[id_].time_cloud_input = timestamp.toSec();

	// add piror factor
	Pose3 new_pose_to;
	int poses_num = initial_values->size();
	Symbol current_symbol = Symbol('a'+id_, poses_num);
	if(poses_num == 0)
	{
		// save piror value
		robots[id_].piror_odom = pose_to;

		// add piror factor to graph
		auto prior_factor = PriorFactor<Pose3>(current_symbol, pose_to, prior_noise);
		local_pose_graph_no_filtering->add(prior_factor);
		isam2_graph.add(prior_factor);

		// add piror value
		initial_values->insert(current_symbol, pose_to);
		isam2_initial_values.insert(current_symbol, pose_to);
		new_pose_to = pose_to;

		LOG(INFO) << "createPrior:[" << id_ << "]" << new_pose_to.translation().x() << " " << new_pose_to.translation().y()
			<< " " << new_pose_to.translation().z() << new_pose_to.rotation().roll() << " " << new_pose_to.rotation().pitch()
			<< " " << new_pose_to.rotation().yaw() << "." << endl;
	}
	// add odometry factor
	else
	{
		// incremental odom in local frame
		auto pose_from = pclPointTogtsamPose3(keyposes_cloud_6d->points[poses_num - 1]);
		auto pose_increment = pose_from.between(pose_to);
		Symbol previous_symbol = Symbol('a'+id_, poses_num - 1);
		Matrix covariance = odometry_noise->covariance();
		
		// add odometry factor to graph
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
			previous_symbol, current_symbol, pose_increment, odometry_noise));
		local_pose_graph->add(factor);
		local_pose_graph_no_filtering->add(factor);
		isam2_graph.add(factor);

		// add odometry value
		isam2_initial_values.insert(current_symbol, pose_to);
		// incremental odom in global graph frame
		pose_from = initial_values->at<Pose3>(previous_symbol);
		new_pose_to = pose_from * pose_increment;
		initial_values->insert(current_symbol, new_pose_to);

		// save factor in local map (for PCM)
		auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
		robot_local_map.addTransform(*new_factor, covariance);

		LOG(INFO) << "createOdom:[" << id_ << "]" << "[" << poses_num - 1 << "-" << poses_num << "]--["
			<< pose_from.translation().x() << " " << pose_from.translation().y() << " " << pose_from.translation().z()
			<< pose_from.rotation().roll() << " " << pose_from.rotation().pitch() << " " << pose_from.rotation().yaw() << "],["
			<< new_pose_to.translation().x() << " " << new_pose_to.translation().y() << " " << new_pose_to.translation().z()
			<< new_pose_to.rotation().roll() << " " << new_pose_to.rotation().pitch() << " " << new_pose_to.rotation().yaw() << "]." << endl;
	}

	// optimizing
	// isam2_graph.print("GTSAM Graph:\n");
	isam2->update(isam2_graph, isam2_initial_values);
	isam2_graph.resize(0);
	isam2_initial_values.clear();
	isam2_current_estimates = isam2->calculateEstimate();
	isam2_keypose_estimate = isam2_current_estimates.at<Pose3>(current_symbol);

	// save pose in local frame
	static PointPose3D pose_3d;
	pose_3d.x = isam2_keypose_estimate.translation().x();
	pose_3d.y = isam2_keypose_estimate.translation().y();
	pose_3d.z = isam2_keypose_estimate.translation().z();
	pose_3d.intensity = poses_num; // keyframe index
	keyposes_cloud_3d->push_back(pose_3d);

	static PointPose6D pose_6d;
	pose_6d.x = pose_3d.x;
	pose_6d.y = pose_3d.y;
	pose_6d.z = pose_3d.z;
	pose_6d.intensity = pose_3d.intensity;
	pose_6d.roll = isam2_keypose_estimate.rotation().roll();
	pose_6d.pitch = isam2_keypose_estimate.rotation().pitch();
	pose_6d.yaw = isam2_keypose_estimate.rotation().yaw();
	pose_6d.time = robots[id_].time_cloud_input; // keyframe timestamp
	keyposes_cloud_6d->push_back(pose_6d);

	LOG(INFO) << "save:[" << id_ << "]--[" << poses_num << "]--" << isam2_keypose_estimate.translation().x()
		<< " " << isam2_keypose_estimate.translation().y() << " " << isam2_keypose_estimate.translation().z()
		<< " " << isam2_keypose_estimate.rotation().roll() << " " << isam2_keypose_estimate.rotation().pitch()
		<< " " << isam2_keypose_estimate.rotation().yaw() << "." << endl;

	// save path for visualization
	updateLocalPath(pose_6d);
	updateGlobalPath(new_pose_to);
}

bool distributedMapping::saveFrame(
	const Pose3& pose_to)
{
	if(keyposes_cloud_3d->empty())
	{
		return true;
	}

	auto last_keypose = pclPointTogtsamPose3(keyposes_cloud_6d->back());
	auto pose_increment = last_keypose.inverse() * pose_to;

	float x = pose_increment.translation().x();
	float y = pose_increment.translation().y();
	float z = pose_increment.translation().z();
	float roll = pose_increment.rotation().roll();
	float pitch = pose_increment.rotation().pitch();
	float yaw = pose_increment.rotation().yaw();

	// select keyframe
	if(abs(roll) < keyframe_angle_threshold_ && abs(pitch) < keyframe_angle_threshold_ && 
		abs(yaw) < keyframe_angle_threshold_ && sqrt(x*x + y*y + z*z) < keyframe_distance_threshold_)
	{
		return false;
	}

	return true;
}

void distributedMapping::updateLocalPath(
	const PointPose6D& pose)
{
	static geometry_msgs::PoseStamped pose_stamped_msg;
	pose_stamped_msg.header.stamp = ros::Time::now();
	pose_stamped_msg.header.frame_id = world_frame_;
	pose_stamped_msg.pose.position.x = pose.x;
	pose_stamped_msg.pose.position.y = pose.y;
	pose_stamped_msg.pose.position.z = pose.z;
	tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
	pose_stamped_msg.pose.orientation.x = q.x();
	pose_stamped_msg.pose.orientation.y = q.y();
	pose_stamped_msg.pose.orientation.z = q.z();
	pose_stamped_msg.pose.orientation.w = q.w();

	local_path.poses.push_back(pose_stamped_msg);
}

bool distributedMapping::updatePoses()
{
	bool return_value = false;

	if(keyposes_cloud_3d->empty())
	{
		return return_value;
	}

	if(intra_robot_loop_close_flag)
	{
		// clear path
		local_path.poses.clear();

		// aggregate estimates
		for(const Values::ConstKeyValuePair &key_value: isam2_current_estimates)
		{
			// update key poses
			Symbol key = key_value.key;
			int index = key.index();
			Pose3 pose = isam2_current_estimates.at<Pose3>(key);

			keyposes_cloud_3d->points[index].x = pose.translation().x();
			keyposes_cloud_3d->points[index].y = pose.translation().y();
			keyposes_cloud_3d->points[index].z = pose.translation().z();

			keyposes_cloud_6d->points[index].x = keyposes_cloud_3d->points[index].x;
			keyposes_cloud_6d->points[index].y = keyposes_cloud_3d->points[index].y;
			keyposes_cloud_6d->points[index].z = keyposes_cloud_3d->points[index].z;
			keyposes_cloud_6d->points[index].roll = pose.rotation().roll();
			keyposes_cloud_6d->points[index].pitch = pose.rotation().pitch();
			keyposes_cloud_6d->points[index].yaw = pose.rotation().yaw();
			

			updateLocalPath(keyposes_cloud_6d->points[index]);
		}
		
		intra_robot_loop_close_flag = false;
		return_value = true;
		LOG(INFO) << "updatePoses:[" << id_ << "]." << endl;
	}

	// copy keyposes
	*copy_keyposes_cloud_3d = *keyposes_cloud_3d;
	*copy_keyposes_cloud_6d = *keyposes_cloud_6d;

	// save updated keypose
	isam2_keypose_estimate = pclPointTogtsamPose3(keyposes_cloud_6d->back());

	return return_value;
}

void distributedMapping::makeDescriptors()
{
	while (!store_descriptors.empty())
	{
		auto msg_data = store_descriptors.front().second;
		auto msg_id = store_descriptors.front().first;
		store_descriptors.pop_front();
		keyframe_descriptor->saveDescriptorAndKey(msg_data.values.data(), msg_id, msg_data.index);
	}

	if(initial_values->empty() || (!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_))
	{
		return;
	}

	// downsample keyframe
	cloud_for_decript_ds->clear();
	if(sensor_ == LiDARType::LIVOX)
	{
		// pcl::PointCloud<PointPose3D>::Ptr multiKeyFrameCloud(new pcl::PointCloud<PointPose3D>());
		// loopFindNearKeyframes(multiKeyFrameCloud, initial_values->size() - 1, history_keyframe_search_num_);
		// downsample_filter_for_descriptor.setInputCloud(multiKeyFrameCloud);
		downsample_filter_for_descriptor.setInputCloud(robots[id_].keyframe_cloud);
	}
	else
	{
		downsample_filter_for_descriptor.setInputCloud(robots[id_].keyframe_cloud);
	}
	downsample_filter_for_descriptor.filter(*cloud_for_decript_ds);

	// make and save global descriptors
	auto descriptor_vec = keyframe_descriptor->makeAndSaveDescriptorAndKey(*cloud_for_decript_ds, id_, initial_values->size()-1);
	LOG(INFO) << "makeDescriptors[" << id_ << "]." << endl;

	// extract descriptors values
	global_descriptor_msg.values.swap(descriptor_vec);
	// keyfame index
	global_descriptor_msg.index = initial_values->size()-1;
	global_descriptor_msg.header.stamp = robots[id_].time_cloud_input_stamp;
	// publish message
	robots[id_].pub_descriptors.publish(global_descriptor_msg);
}

void distributedMapping::publishPath()
{
	// publish path
	if(pub_global_path.getNumSubscribers() != 0)
	{
		global_path.header.stamp = robots[id_].time_cloud_input_stamp;
		global_path.header.frame_id = world_frame_;
		pub_global_path.publish(global_path);
	}

	if(pub_local_path.getNumSubscribers() != 0)
	{
		local_path.header.stamp = robots[id_].time_cloud_input_stamp;
		local_path.header.frame_id = robots[id_].odom_frame_;
		pub_local_path.publish(local_path);
	}
}

void distributedMapping::publishTransformation(
	const ros::Time& timestamp)
{
	static tf::TransformBroadcaster world_to_odom_tf_broadcaster;
	static Symbol first_key((id_+'a'), 0);

	Pose3 first_pose = initial_values->at<Pose3>(first_key);
	Pose3 old_first_pose = robots[id_].piror_odom;
	Pose3 pose_between = first_pose * old_first_pose.inverse();

	tf::Transform world_to_odom = tf::Transform(tf::createQuaternionFromRPY(
		pose_between.rotation().roll(), pose_between.rotation().pitch(), pose_between.rotation().yaw()),
		tf::Vector3(pose_between.translation().x(), pose_between.translation().y(), pose_between.translation().z()));

	world_to_odom_tf_broadcaster.sendTransform(tf::StampedTransform(world_to_odom, timestamp, world_frame_, robots[id_].odom_frame_));
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: distributed mapping
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
vector<int32_t> distributedMapping::roundMembers() const
{
	set<int> connected{id_};
	bool changed = true;
	while(changed)
	{
		changed = false;
		for(const auto& edge : known_inter_loops)
		{
			int a = dcl_slam::OptimizationRound::robot(edge.first);
			int b = dcl_slam::OptimizationRound::robot(edge.second);
			if(connected.count(a) || connected.count(b))
			{
				changed |= connected.insert(a).second;
				changed |= connected.insert(b).second;
			}
		}
	}
	return vector<int32_t>(connected.begin(), connected.end());
}

void distributedMapping::recordInterLoop(Key a, Key b)
{
	if(a > b) std::swap(a,b);
	if(Symbol(a).chr() == Symbol(b).chr()) return;
	if(known_inter_loops.emplace(a,b).second)
	{
		connected_members = roundMembers();
		sent_start_optimization_flag = true;
	}
}

void distributedMapping::beginRound(const dcl_slam::optimization_state& proposal)
{
	round.begin(proposal);
	rotation_inbox.clear();
	pose_inbox.clear();
	rotation_outbox.resize(number_of_robots_);
	pose_outbox.resize(number_of_robots_);
	round_send_keys.clear();
	round_receive_keys.clear();
	neighbors_anchor_offset.clear();
	current_rotation_estimate_iteration = current_pose_estimate_iteration = 0;
	round_graph_prepared = round_commit_applied = false;
	rotation_local_converged = pose_local_converged = false;
	round_progress = ros::WallTime::now();
	changeOptimizerState(OptimizerState::Start);
}

void distributedMapping::updateOptimizer()
{
	// Upstream load/reinit retains neighbor caches. Start with a clean round solver.
	optimizer.reset(new distributed_mapper::DistributedMapper(char('a'+id_)));
	optimizer->setUseBetweenNoiseFlag(use_between_noise_);
	optimizer->setUseLandmarksFlag(use_landmarks_);
	optimizer->setVerbosity(distributed_mapper::DistributedMapper::ERROR);
	optimizer->setFlaggedInit(use_flagged_init_);
	optimizer->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
	optimizer->setGamma(gamma_);
	auto graph_values = std::make_pair(round_pose_graph, round_initial_values);
	optimizer->loadSubgraphAndCreateSubgraphEdge(graph_values);
	prior_owner = round.status.coordinator;
	if(prior_owner == id_)
		optimizer->addPrior(Symbol(char('a'+id_), 0), robots[id_].piror_odom, prior_noise);
	optimizer->updateInitialized(false);
	optimizer->clearNeighboringRobotInit();
}

void distributedMapping::outliersFiltering()
{
	measurements_accepted_num = measurements_rejected_num = 0;
	if(!use_pcm_) return;
	set<size_t> rejected;
	for(int neighbor : round.status.participants)
	{
		if(neighbor == id_ || !pose_estimates_from_neighbors.count(neighbor)) continue;
		graph_utils::Transforms empty, transforms;
		auto neighbor_info = robot_measurements::RobotLocalMap(
			pose_estimates_from_neighbors.at(neighbor), empty, round_robot_map.getLoopClosures());
		for(const auto& transform : round_robot_map.getTransforms().transforms)
		{
			int a = int(Symbol(transform.second.i).chr()) - 'a';
			int b = int(Symbol(transform.second.j).chr()) - 'a';
			if(a == neighbor || b == neighbor) transforms.transforms.insert(transform);
		}
		auto measurements = robot_measurements::InterRobotMeasurements(
			transforms, optimizer->robotName(), char('a'+neighbor));
		auto local_info = robot_measurements::RobotLocalMap(
			pose_estimates_from_neighbors.at(id_), round_robot_map.getTransforms(), round_robot_map.getLoopClosures());
		auto map = global_map::GlobalMap(local_info, neighbor_info, measurements, pcm_threshold_, use_heuristics_);
		auto clique = map.pairwiseConsistencyMaximization();
		measurements_accepted_num += clique.first.size();
		measurements_rejected_num += clique.second;
		for(size_t slot : optimizer->loopclosureEdge())
			if(distributed_pcm::DistributedPCM::isloopclosureToBeRejected(
				clique.first, slot, transforms, measurements.getLoopClosures(), optimizer))
				rejected.insert(slot);
	}
	// Preserve factor orientation/noise; only the round snapshot is filtered.
	for(auto it = rejected.rbegin(); it != rejected.rend(); ++it)
		round_pose_graph->erase(round_pose_graph->begin() + *it);
}

void distributedMapping::initializePoseGraphOptimization()
{
	set<pair<Key,Key>> selected;
	for(size_t i = 0; i < round.proposal_edges.size(); i += 2)
		selected.emplace(round.proposal_edges[i], round.proposal_edges[i+1]);
	// Wait for every proposed incident constraint and its owned keyframe.
	for(const auto& edge : selected)
	{
		for(Key key : {edge.first, edge.second})
			if(Symbol(key).chr() == char('a'+id_) &&
				(!known_inter_loops.count(edge) || !initial_values->exists(key))) return;
	}
	round_pose_graph = boost::make_shared<NonlinearFactorGraph>();
	round_initial_values = boost::make_shared<Values>(*initial_values);
	round_robot_map = robot_local_map;
	for(const auto& factor : *local_pose_graph_no_filtering)
	{
		if(!factor || factor->keys().size() != 2) continue; // add the agreed prior once below
		Key a = factor->keys()[0], b = factor->keys()[1];
		if(Symbol(a).chr() != Symbol(b).chr() && !selected.count(std::minmax(a,b)))
		{
			round_robot_map.removeTransform(std::make_pair(a,b));
			continue;
		}
		round_pose_graph->push_back(factor);
	}
	sent_start_optimization_flag = false;
	updateOptimizer();
	outliersFiltering();
	set<pair<Key,Key>> retained;
	for(const auto& factor : *round_pose_graph)
		if(factor->keys().size() == 2 && Symbol(factor->keys()[0]).chr() != Symbol(factor->keys()[1]).chr())
			retained.insert(std::minmax(factor->keys()[0], factor->keys()[1]));
	round.status.separator_keys.clear();
	for(const auto& edge : retained)
	{
		round.status.separator_keys.push_back(edge.first);
		round.status.separator_keys.push_back(edge.second);
	}
	round_graph_prepared = true;
	ROS_INFO("Round %llu robot %d prepared, local PCM retained %zu edges",
		(unsigned long long)round.status.round_id, id_, round.status.separator_keys.size()/2);
	changeOptimizerState(OptimizerState::Initialization);
}

bool distributedMapping::applyRoundCommit(const dcl_slam::optimization_state& commit)
{
	if(!round.same(commit) || !commit.committed || commit.aborted || !round_graph_prepared) return false;
	set<pair<Key,Key>> selected;
	for(size_t i = 0; i < commit.separator_keys.size(); i += 2)
		selected.emplace(commit.separator_keys[i], commit.separator_keys[i+1]);
	for(const auto& edge : selected)
	{
		if(Symbol(edge.first).chr() != char('a'+id_) && Symbol(edge.second).chr() != char('a'+id_)) continue;
		bool found = false;
		for(const auto& factor : *round_pose_graph)
			if(factor->keys().size() == 2 &&
				std::minmax(factor->keys()[0], factor->keys()[1]) == std::minmax(edge.first, edge.second)) found = true;
		if(!found) throw std::runtime_error("Commit includes a locally PCM-rejected separator");
	}
	for(size_t i = round_pose_graph->size(); i > 0; --i)
	{
		const auto& keys = round_pose_graph->at(i-1)->keys();
		if(keys.size() == 2 && Symbol(keys[0]).chr() != Symbol(keys[1]).chr() &&
			!selected.count(std::minmax(keys[0],keys[1])))
			round_pose_graph->erase(round_pose_graph->begin()+i-1);
	}
	round.status.separator_keys = commit.separator_keys;
	round.status.order = commit.order;
	round.status.committed = true;
	for(const auto& edge : selected)
	{
		Key local = 0, remote = 0;
		if(Symbol(edge.first).chr() == char('a'+id_)) {local=edge.first; remote=edge.second;}
		if(Symbol(edge.second).chr() == char('a'+id_)) {local=edge.second; remote=edge.first;}
		if(local)
		{
			int other = dcl_slam::OptimizationRound::robot(remote);
			round_send_keys[other].push_back(local);
			round_receive_keys[other].push_back(remote);
		}
	}
	for(auto* table : {&round_send_keys, &round_receive_keys})
		for(auto& entry : *table)
		{
			auto& keys = entry.second;
			std::sort(keys.begin(),keys.end());
			keys.erase(std::unique(keys.begin(),keys.end()),keys.end());
		}
	updateOptimizer();
	round_commit_applied = true;
	publishRoundState();
	return true;
}

void distributedMapping::abortOptimization(const bool& log_info)
{
	if(log_info) ROS_WARN("Abort round %llu on robot %d", (unsigned long long)round.status.round_id, id_);
	round.status.aborted = true;
	sent_start_optimization_flag = true;
	rotation_inbox.clear();
	pose_inbox.clear();
	changeOptimizerState(OptimizerState::Idle);
}

void distributedMapping::failSafeCheck()
{
	if(optimizer_state != OptimizerState::Idle &&
		(ros::WallTime::now() - round_progress).toSec() > round_timeout)
	{
		ROS_WARN("Round %llu timed out in state %d on robot %d",
			(unsigned long long)round.status.round_id, int(optimizer_state), id_);
		abortOptimization(true);
	}
}

void distributedMapping::initializePoseEstimation()
{
	optimizer->convertLinearizedRotationToPoses();
	Values neighbors = optimizer->neighbors();
	for(const Values::ConstKeyValuePair& key_value: neighbors)
	{
		Key key = key_value.key;
		// pick linear rotation estimate
		VectorValues neighbor_estimate_rot_lin;
		neighbor_estimate_rot_lin.insert(key,  optimizer->neighborsLinearizedRotationsAt(key));
		// make a pose out of it
		Values neighbor_rot_estimate = 
			InitializePose3::normalizeRelaxedRotations(neighbor_estimate_rot_lin);
		Values neighbor_pose_estimate = 
			distributed_mapper::evaluation_utils::pose3WithZeroTranslation(neighbor_rot_estimate);
		// store it
		optimizer->updateNeighbor(key, neighbor_pose_estimate.at<Pose3>(key));
	}

	// reset flags for flagged initialization.
	optimizer->updateInitialized(false);
	optimizer->clearNeighboringRobotInit();
	optimizer->resetLatestChange();
	pose_local_converged = false;
}


void distributedMapping::updateGlobalPath(
	const Pose3& pose)
{
	static geometry_msgs::PoseStamped pose_stamped_msg;
	pose_stamped_msg.header.stamp = ros::Time::now();
	pose_stamped_msg.header.frame_id = world_frame_;
	pose_stamped_msg.pose.position.x = pose.translation().x();
	pose_stamped_msg.pose.position.y = pose.translation().y();
	pose_stamped_msg.pose.position.z = pose.translation().z();
	pose_stamped_msg.pose.orientation.x = pose.rotation().toQuaternion().x();
	pose_stamped_msg.pose.orientation.y = pose.rotation().toQuaternion().y();
	pose_stamped_msg.pose.orientation.z = pose.rotation().toQuaternion().z();
	pose_stamped_msg.pose.orientation.w = pose.rotation().toQuaternion().w();

	global_path.poses.push_back(pose_stamped_msg);
}

void distributedMapping::incrementalInitialGuessUpdate()
{
	// update poses values
	initial_values->update(optimizer->currentEstimate());

	// incremental update
	for(size_t k = 0; k < local_pose_graph->size(); k++)
	{
		KeyVector keys = local_pose_graph->at(k)->keys();
		if(keys.size() == 2)
		{
			auto between_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(local_pose_graph->at(k));
			Symbol key0 = between_factor->keys().at(0);
			Symbol key1 = between_factor->keys().at(1);
			int index0 = key0.index();
			int robot0 = key0.chr();
			int index1 = key1.index();
			int robot1 = key1.chr();
			if(!optimizer->currentEstimate().exists(key1) && (robot0 == robot1) && (index1 == index0 + 1))
			{
				// get previous pose
				auto previous_pose = initial_values->at<Pose3>(key0);
				// compose previous pose and measurement
				auto current_pose = previous_pose * between_factor->measured();
				// update pose in initial guess
				initial_values->update(key1, current_pose);
			}
		}
	}

	
	// aggregate estimates
	global_path.poses.clear();
	pcl::PointCloud<PointPose3D>::Ptr poses_3d_cloud(new pcl::PointCloud<PointPose3D>());
	for(const Values::ConstKeyValuePair &key_value: *initial_values)
	{
		// update key poses
		Symbol key = key_value.key;
		int index = key.index();
		Pose3 pose = initial_values->at<Pose3>(key);

		PointPose3D pose_3d;
		pose_3d.x = pose.translation().x();
		pose_3d.y = pose.translation().y();
		pose_3d.z = pose.translation().z();
		pose_3d.intensity = key.index();
		poses_3d_cloud->push_back(pose_3d);

		updateGlobalPath(pose);
	}

	if(pub_keypose_cloud.getNumSubscribers() != 0)
	{
		// publish global map
		sensor_msgs::PointCloud2 keypose_cloud_msg;
		pcl::toROSMsg(*poses_3d_cloud, keypose_cloud_msg);
		keypose_cloud_msg.header.stamp = ros::Time::now();
		keypose_cloud_msg.header.frame_id = world_frame_;
		pub_keypose_cloud.publish(keypose_cloud_msg);
	}

	if(pub_global_path.getNumSubscribers() != 0)
	{
		global_path.header.stamp = ros::Time::now();
		global_path.header.frame_id = world_frame_;
		pub_global_path.publish(global_path);
	}
}

void distributedMapping::endOptimization()
{
	// retract to global frame
	if(prior_owner == id_)
	{
		optimizer->retractPose3GlobalWithOffset(anchor_offset);
	}
	else
	{
		optimizer->retractPose3GlobalWithOffset(neighbors_anchor_offset[prior_owner]);
	}

	// update estimates
	incrementalInitialGuessUpdate();

}

void distributedMapping::publishRoundState()
{
	round.status.header.stamp = ros::Time::now();
	++round.status.header.seq;
	round.status.state = int(optimizer_state);
	robots[id_].pub_optimization_state.publish(round.status);
	last_round_publish = ros::WallTime::now();
}

void distributedMapping::changeOptimizerState(const OptimizerState& state)
{
	optimizer_state = state;
	round_progress = ros::WallTime::now();
	ROS_INFO("Round %llu robot %d state %d committed=%d",
		(unsigned long long)round.status.round_id, id_, int(state), int(round.status.committed));
	publishRoundState();
}

void distributedMapping::run(const ros::TimerEvent&)
{
	try
	{
		if(round.status.aborted && optimizer_state != OptimizerState::Idle) abortOptimization(false);
		switch(optimizer_state)
		{
			case OptimizerState::Idle:
			{
				const auto& members = connected_members;
				if(sent_start_optimization_flag && members.size() > 1 && members.front() == id_ &&
					(ros::WallTime::now()-round_progress).toSec() > 1.0)
				{
					dcl_slam::optimization_state proposal;
					proposal.round_id = std::max(ros::WallTime::now().toNSec(), round.status.round_id + 1);
					proposal.coordinator = id_;
					proposal.participants = members;
					for(const auto& edge : known_inter_loops)
						if(dcl_slam::OptimizationRound::contains(members, dcl_slam::OptimizationRound::robot(edge.first)) &&
							dcl_slam::OptimizationRound::contains(members, dcl_slam::OptimizationRound::robot(edge.second)))
						{
							proposal.separator_keys.push_back(edge.first);
							proposal.separator_keys.push_back(edge.second);
						}
					proposal.proposal_keys = proposal.separator_keys;
					beginRound(proposal);
				}
				break;
			}
			case OptimizerState::Start:
				initializePoseGraphOptimization();
				break;
			case OptimizerState::Initialization:
			{
				if(id_ == round.status.coordinator && !round.status.committed && round.allPrepared(id_))
				{
					auto commit = round.status;
					commit.separator_keys = round.agreedEdges(id_);
					commit.order = dcl_slam::OptimizationRound::connectedOrder(commit.participants,commit.separator_keys);
					if(commit.order.empty()) throw std::runtime_error("PCM-consistent round graph is disconnected");
					commit.committed = true;
					applyRoundCommit(commit);
				}
				if(id_ != round.status.coordinator && !round_commit_applied)
				{
					auto leader = round.peers.find(round.status.coordinator);
					if(leader != round.peers.end() && leader->second->committed) applyRoundCommit(*leader->second);
				}
				bool start = id_ == round.status.coordinator ? round.allCommitted(id_) : false;
				auto leader = round.peers.find(round.status.coordinator);
				if(id_ != round.status.coordinator && leader != round.peers.end())
					start = round.matchingCommit(*leader->second) && leader->second->state >= int(OptimizerState::RotationEstimation);
				if(round_commit_applied && start)
				{
					changeOptimizerState(OptimizerState::RotationEstimation);
					publishEstimates(true, true);
				}
				break;
			}
			case OptimizerState::RotationEstimation:
				estimateStep(true);
				if(round.phaseDone(id_,true) && applyNeighborEstimates(true,true))
					changeOptimizerState(OptimizerState::PoseEstimationInitialization);
				break;
			case OptimizerState::PoseEstimationInitialization:
				initializePoseEstimation();
				anchor_offset = Point3::Zero();
				changeOptimizerState(OptimizerState::PoseEstimation);
				publishEstimates(false,true);
				break;
			case OptimizerState::PoseEstimation:
				estimateStep(false);
				if(round.phaseDone(id_,false) && applyNeighborEstimates(false,true))
				{
					endOptimization();
					changeOptimizerState(OptimizerState::End);
				}
				break;
			case OptimizerState::End:
				changeOptimizerState(OptimizerState::PostEndingCommunicationDelay);
				break;
			case OptimizerState::PostEndingCommunicationDelay:
				if(round.allEnded(id_)) changeOptimizerState(OptimizerState::Idle);
				break;
		}
		if((ros::WallTime::now()-last_round_publish).toSec() >= 0.25)
		{
			publishRoundState();
			if(round_commit_applied && !round.status.aborted && optimizer_state >= OptimizerState::RotationEstimation)
				publishEstimates(true);
			if(round_commit_applied && !round.status.aborted && optimizer_state >= OptimizerState::PoseEstimation)
				publishEstimates(false);
		}
		failSafeCheck();
	}
	catch(const std::exception& error)
	{
		ROS_ERROR("Round %llu robot %d failed: %s", (unsigned long long)round.status.round_id, id_, error.what());
		abortOptimization(true);
	}
}