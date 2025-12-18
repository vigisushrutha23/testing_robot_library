/**
 * @file    differential_drive_feedback.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Numerical simulation to test nonlinear feedback control of differential drive class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Eigen/Core>
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>  
#include <RobotLibrary/Control/DifferentialDriveFeedback.h>
#include <RobotLibrary/Math/Ellipsoid.h>
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/MinimumArcLength.h>

// Simulation parameters
double controlFrequency  = 100;
double simulationTime = 10.0;
int simulationSteps =  static_cast<int>(controlFrequency * simulationTime);

int main(int argc, char **argv)
{   
    // Set up the trajectory
    RobotLibrary::Model::Pose2D startPose(0.0, 0.0, 0.0);
    Eigen::Vector2d endPoint = {-1.0, 1.0};
    RobotLibrary::Trajectory::MinimumArcLength trajectory(startPose, endPoint, 1.0, simulationTime - 1.0);
    
    // Parameters for the model
    RobotLibrary::Model::DifferentialDriveParameters modelParameters;
    modelParameters.inertia                = 0.5 * 5.0 * 0.25 * 0.25;                               // Rotational inertia (kg*m^2)
    modelParameters.mass                   = 5.0;                                                   // Weight (kg)
    modelParameters.maxAngularAcceleration = 5.0;                                                   // Maximum rotational acceleration (rad/s/s)
    modelParameters.maxAngularVelocity     = 100.0 * M_PI / 30.0;                                   // Maximum rotational speed (rad/s)
    modelParameters.maxLinearAcceleration  =  3.0;                                                  // Maximum forward acceleration (m/s/s)
    modelParameters.maxLinearVelocity      =  2.0;                                                  // Maximum forward speed (m/s)
    modelParameters.propagationUncertainty = Eigen::Matrix3d::Identity();                           // Uncertainty of configuration propagation in Kalman filter
    
    // Parameters for the feedback controller
    RobotLibrary::Control::DifferentialDriveFeedbackParameters controlParameters;
    controlParameters.controlFrequency    = controlFrequency;
    controlParameters.minimumSafeDistance =  0.1;
    controlParameters.orientationGain     = 10.0;
    controlParameters.xPositionGain       =  5.0;
    controlParameters.yPositionGain       = 50.0;
    
    // These are for the QP solver
    controlParameters.qpsolver.stepSizeTolerance = 1e-08;                                           // Needs to be very small for this low dimensional problem

    RobotLibrary::Control::DifferentialDriveFeedback controller(modelParameters, controlParameters);
    
    // Set up the obstacles
    auto line = std::make_unique<RobotLibrary::Math::Line2D>(Eigen::Vector2d(0, 1.0));
   
    auto obstacle = RobotLibrary::Model::Obstacle2D(std::move(line));                               // Create obstacle
    
    obstacle.update_state(RobotLibrary::Model::Pose2D(0.2, 0.0, 0.0), Eigen::Vector3d::Zero());     // Set new pose (zero speed)
    
    std::vector<RobotLibrary::Model::Obstacle2D> obstacles;
    obstacles.push_back(std::move(obstacle));
    
    // Set up data arrays
    std::vector<std::array<double,3>> desiredConfiguration(simulationSteps);
    std::vector<std::array<double,3>> actualConfiguration(simulationSteps);
    std::vector<std::array<double,2>> poseError(simulationSteps);
    std::vector<std::array<double,2>> controlInputs(simulationSteps);
    
    RobotLibrary::Model::Pose2D actualPose(0.0, 0.0, 0.0);                                        // Start offset from the trajectory
    
    Eigen::Vector2d controlInput = {0.0, 0.0};
    
    controller.update_state(actualPose, controlInput);
    
    // Run the simulation
    for (int i = 0; i < simulationSteps; ++i)
    {
        
        double simTime = i / controlFrequency;
        
        // Solve the trajectory tracking problem
        const auto &[desiredPosition,
                     desiredVelocity,
                     desiredAcceleration] = trajectory.query_state(simTime);                        // Get the desired state
                     
        RobotLibrary::Model::Pose2D desiredPose(desiredPosition[0],
                                                desiredPosition[1],
                                                desiredPosition[2]);                                // We need to put it in a Pose2D object
        
        // NOTE: QP solver can throw an error if no solution exists.
        try
        {
            controlInput = controller.track_trajectory(desiredPose, desiredVelocity, obstacles);
        }
        catch (const std::exception &exception)
        {
            std::cout << exception.what() << "\n";
            
            break;
        }
        
        // Save data for analysis
        desiredConfiguration[i] = {desiredPosition[0], desiredPosition[1], desiredPosition[2]};
        actualConfiguration[i]  = {actualPose.translation()[0], actualPose.translation()[1], actualPose.angle()};
        poseError[i]            = {(desiredPose.translation() - controller.pose().translation()).norm(), abs(desiredPose.angle() - controller.pose().angle())};                   
        controlInputs[i]        = {controlInput[0], controlInput[1]};
        
        // Propagate the pose
        controller.update_state(actualPose, controlInput);
        actualPose = controller.predicted_pose();                                                   // Assume perfect tracking
    }
    
    std::ofstream file;

    // Save the trajectory data
    file.open("desired_configuration_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << desiredConfiguration[i][j];
      file << "\n";
    }
    file.close();
    
    // Save the actual configuration data
    file.open("actual_configuration_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << actualConfiguration[i][j];
      file << "\n";
    }
    file.close();  
    
    // Save the control data
    file.open("control_input_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 2; ++j) file << "," << controlInputs[i][j];
      file << "\n";
    }
    file.close();  
    
    // Save the error data
    file.open("tracking_error_data.csv");
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency);
        for(int j = 0; j < 2; ++j) file << "," << poseError[i][j];
        file << "\n";
    }
    file.close(); 
    
    std::cout << "[INFO] [DIFFERENTIAL DRIVE FEEDBACK CONTROL] Numerical simulation complete. "
              << "Data saved to .csv files for analysis.\n";
    
    return 0;                                                                                       // No problems with main
}
