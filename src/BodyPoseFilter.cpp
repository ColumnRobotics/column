#include <column/BodyPoseFilter.h>
#include <time.h>


geometry_msgs::PoseWithCovarianceStamped BodyPoseFilter::ransac_line(const std::deque<geometry_msgs::PoseStamped> &  reading_vec){
    // Parameters
    srand(time(NULL)); // Seed the random number generator
    int num_runs = 20; // How many times to try ransac
    int vec_size = reading_vec.size(); // Number of readings
    double th = 0.1; // Threshold for an inlier

    int best_inliers = 0; // Max inliers
    double best_avg[3] = {0, 0, 0}; // Average values
    double sum[3] = {0, 0, 0}; // Average values
    ros::Time time_sum;
    time_sum.sec = 0;
    time_sum.nsec = 0;
    ros::Time best_average_time = time_sum;
    double pose_dot, line_dot;

    /**
    * Run ranac
    */
    for(int i = 0; i < num_runs; i++){
        // Get the indices
        int pose1_index = 0 + (rand() % (int)(vec_size));
        int pose2_index = 0 + (rand() % (int)(vec_size));
        sum[0] = 0;
        sum[1] = 0;
        sum[2] = 0;
        // Get the pose
        geometry_msgs::PoseStamped pose1 = reading_vec[pose1_index];
        geometry_msgs::PoseStamped pose2 = reading_vec[pose2_index];

        // Get the line between the poses
        geometry_msgs::PoseStamped line;
        line.pose.position.x = pose1.pose.position.x - pose2.pose.position.x;
        line.pose.position.y = pose1.pose.position.y - pose2.pose.position.y;
        line.pose.position.z = pose1.pose.position.z - pose2.pose.position.z;

        // Count the number of inliers
        int inliers = 0;
        geometry_msgs::PoseStamped curr_pose;
        geometry_msgs::PoseStamped projected_pose;
        for(int j = 0; j < vec_size; j++){
            if(j != pose1_index || j != pose2_index){ // If it is not part of the line
                // Get the projected pose
                curr_pose = reading_vec[j];
                pose_dot = curr_pose.pose.position.x * line.pose.position.x +
                           curr_pose.pose.position.y * line.pose.position.y +
                           curr_pose.pose.position.z * line.pose.position.z;
                line_dot = line.pose.position.x * line.pose.position.x +
                           line.pose.position.y * line.pose.position.y +
                           line.pose.position.z * line.pose.position.z;
                projected_pose.pose.position.x = curr_pose.pose.position.x * (pose_dot / line_dot);
                projected_pose.pose.position.y = curr_pose.pose.position.y * (pose_dot / line_dot);
                projected_pose.pose.position.z = curr_pose.pose.position.z * (pose_dot / line_dot);

                // Get the distance to the line
                double dist = std::sqrt(std::pow(projected_pose.pose.position.x - curr_pose.pose.position.x, 2) +
                    std::pow(projected_pose.pose.position.y - curr_pose.pose.position.y, 2) +
                    std::pow(projected_pose.pose.position.z - curr_pose.pose.position.z, 2));

                // Check to see if inlier
                if(dist < th){
                    sum[0] += curr_pose.pose.position.x;
                    sum[1] += curr_pose.pose.position.y;
                    sum[2] += curr_pose.pose.position.z;
                    inliers += 1;
                    time_sum.sec = time_sum.sec + curr_pose.header.stamp.sec;
                    time_sum.nsec = time_sum.nsec + curr_pose.header.stamp.nsec;
                }
            }
        }
        if(inliers > best_inliers){
            best_inliers = inliers;
            best_avg[0] = sum[0] / best_inliers;
            best_avg[1] = sum[1] / best_inliers;
            best_avg[2] = sum[2] / best_inliers;
            best_average_time.sec = time_sum.sec / best_inliers;
            best_average_time.nsec = time_sum.nsec / best_inliers;
        }
    }

    // Return the filtered estimate
    geometry_msgs::PoseWithCovarianceStamped filtered_estimate;
    filtered_estimate.pose.pose.position.x = best_avg[0];
    filtered_estimate.pose.pose.position.y = best_avg[1];
    filtered_estimate.pose.pose.position.z = best_avg[2];
    filtered_estimate.pose.pose.orientation.x = 0; 
    filtered_estimate.pose.pose.orientation.y = 0; 
    filtered_estimate.pose.pose.orientation.z = 0; 
    filtered_estimate.pose.pose.orientation.w = 0; 
    filtered_estimate.header.stamp = best_average_time;

    double variance = (vec_size - best_inliers) / vec_size;
    for(int i = 0; i < 36; i++){
        if(i % 7 == 0){
            filtered_estimate.pose.covariance[i] = variance;
        }
        else{
            filtered_estimate.pose.covariance[i] = 0;
        }
    }
    return filtered_estimate;
}

geometry_msgs::PoseWithCovarianceStamped BodyPoseFilter::ransac_point(const std::deque<geometry_msgs::PoseStamped> &  reading_vec){
    // Parameters
    int vec_size = reading_vec.size(); // Number of readings
    int num_runs = vec_size; // How many times to try ransac
    double th = 0.1; // Threshold for an inlier

    int best_inliers = 0; // Max inliers
    double best_avg[3] = {0, 0, 0}; // Average values
    double sum[3] = {0, 0, 0}; // Average values
    ros::Time time_sum;
    time_sum.sec = 0;
    time_sum.nsec = 0;
    ros::Time best_average_time = time_sum;
    double pose_dot, line_dot;
    geometry_msgs::PoseStamped curr_pose, pose;
    /**
    * Run ranac
    */
    for(int i = 0; i < num_runs; i++){
        // Get the indices
        int pose_index = i;
        sum[0] = 0;
        sum[1] = 0;
        sum[2] = 0;
        // Get the pose
        pose = reading_vec[pose_index];

        // Count the number of inliers
        int inliers = 0;
        
        for(int j = 0; j < vec_size; j++){
            if(j != pose_index){ // If it is not part of the line
                // Get the projected pose
                curr_pose = reading_vec[j];         
                // Get the distance to the line
                double dist = std::sqrt(std::pow(pose.pose.position.x - curr_pose.pose.position.x, 2) +
                    std::pow(pose.pose.position.y - curr_pose.pose.position.y, 2) +
                    std::pow(pose.pose.position.z - curr_pose.pose.position.z, 2));

                // Check to see if inlier
                if(dist < th){
                    sum[0] += curr_pose.pose.position.x;
                    sum[1] += curr_pose.pose.position.y;
                    sum[2] += curr_pose.pose.position.z;
                    inliers += 1;
                    time_sum.sec = time_sum.sec + curr_pose.header.stamp.sec;
                    time_sum.nsec = time_sum.nsec + curr_pose.header.stamp.nsec;
                }
            }
        }
        if(inliers > best_inliers){
            best_inliers = inliers;
            best_avg[0] = sum[0] / best_inliers;
            best_avg[1] = sum[1] / best_inliers;
            best_avg[2] = sum[2] / best_inliers;
            best_average_time.sec = time_sum.sec / best_inliers;
            best_average_time.nsec = time_sum.nsec / best_inliers;
        }
    }

    // Return the filtered estimate
    geometry_msgs::PoseWithCovarianceStamped filtered_estimate;
    filtered_estimate.pose.pose.position.x = best_avg[0];
    filtered_estimate.pose.pose.position.y = best_avg[1];
    filtered_estimate.pose.pose.position.z = best_avg[2];
    filtered_estimate.pose.pose.orientation.x = 0; 
    filtered_estimate.pose.pose.orientation.y = 0; 
    filtered_estimate.pose.pose.orientation.z = 0; 
    filtered_estimate.pose.pose.orientation.w = 0; 
    filtered_estimate.header.stamp = best_average_time;
    double variance = ((double)vec_size - (double)best_inliers) / (double)vec_size;
    for(int i = 0; i < 36; i++){
        if(i % 7 == 0){
            filtered_estimate.pose.covariance[i] = variance;
        }
        else{
            filtered_estimate.pose.covariance[i] = 0;
        }
    }
    ROS_INFO("best_iniliers: %d variance: %f covariance[0]: %f \n", best_inliers, variance, filtered_estimate.pose.covariance[0]);
    return filtered_estimate;
}

/**
* Returns the pose of the body frame in the AprilTag frame using a better estimate of the orientation from the IMU 
*/
geometry_msgs::PoseStamped BodyPoseFilter::rotation_invariance_imu(const geometry_msgs::PoseStamped & april_tag_in_camera, 
    const geometry_msgs::PoseStamped & body_pose_in_local){}
