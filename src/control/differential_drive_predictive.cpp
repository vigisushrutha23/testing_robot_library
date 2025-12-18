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
#include <RobotLibrary/Control/DifferentialDrivePredictive.h>
#include <RobotLibrary/Math/Line.h>
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/MinimumArcLength.h>

// Simulation parameters
double       simulationTime   =  10.0;
double       controlFrequency = 100.0;

unsigned int simulationSteps  = 1000;
unsigned int predictionSteps  =   50;

int main(int argc, char **argv)
{   
    // Set up the trajectory
    RobotLibrary::Model::Pose2D startPose(0.0, 0.0, 0.0);
    Eigen::Vector2d endPoint = {-1.0, 1.0};
    RobotLibrary::Trajectory::MinimumArcLength trajectory(startPose, endPoint, 1.0, simulationTime - 1.0);
    
    // Parameters for the model
    RobotLibrary::Model::DifferentialDriveParameters modelParameters;
    modelParameters.inertia                = 0.5 * 50.0 * 0.25 * 0.25;                              // Rotational inertia (kg*m^2)
    modelParameters.mass                   = 50.0;                                                  // Weight (kg)
    modelParameters.maxAngularAcceleration = 0.5;                                                   // Maximum rotational acceleration (rad/s/s)
    modelParameters.maxAngularVelocity     = 100.0 * M_PI / 30.0;                                   // Maximum rotational speed (rad/s)
    modelParameters.maxLinearAcceleration  = 0.5;                                                  // Maximum forward acceleration (m/s/s)
    modelParameters.maxLinearVelocity      = 2.0;                                                  // Maximum forward speed (m/s)
    modelParameters.minimumSafeDistance    = 0.5;
    modelParameters.propagationUncertainty = Eigen::Matrix3d::Identity();                           // Uncertainty of configuration propagation in Kalman filter
    
    // Parameters for the predictive controller
    RobotLibrary::Control::DifferentialDrivePredictiveParameters controlParameters;
    controlParameters.controlFrequency        = controlFrequency;
    controlParameters.exponent                =  0.01;                                              // Growth or decay of pose error weighting
    controlParameters.maximumControlStepNorm  = 1e-06;                                              // DDP algorithm terminates early if max. ||du|| is smaller than this
    controlParameters.numberOfRecursions      = 25;                                                 // No. of forward & backward passes for the DDP algorithm
    controlParameters.obstaclePotentialScalar = 5e-03;                                              // Scales the repulsion force
    controlParameters.predictionSteps         = predictionSteps;                                    // Length of prediction horizon
   
    controlParameters.poseErrorWeight << 2000.0,    0.0,  0.0,
                                            0.0, 2000.0,  1.0,
                                            0.0,    1.0,  5.0;
    
    SolverOptions<double> solverOptions;                                                            // Not currently being used
    
    RobotLibrary::Control::DifferentialDrivePredictive controller(modelParameters,
                                                                  controlParameters,
                                                                  solverOptions);
 
    RobotLibrary::Model::Pose2D actualPose(-0.1, 0.1, 0.0);                                          // Start offset from the trajectory
    
    Eigen::Vector2d controlInput = {0.0, 0.0};
    
    controller.update_state(actualPose, controlInput);
    
    // Set up obstacle(s)
    std::vector<std::vector<RobotLibrary::Model::Obstacle2D>> obstacles(predictionSteps+1);           // MUST be N+1

    for (int i = 0; i < predictionSteps+1; ++i)
    {
        // NOTE: We need N+1 here since for u[0], ... , u[N-1], and x[1], ... , x[N]
        // NOTE: We require the unique_ptr for polymorphism
        
        auto line = std::make_unique<RobotLibrary::Math::Line2D>(Eigen::Vector2d(0.0, 1.0));        // Create line
        
        obstacles[i].push_back(RobotLibrary::Model::Obstacle2D(std::move(line)));                   // Move it in to the obstacle vector
        
        obstacles[i].back().update_state(RobotLibrary::Model::Pose2D(0.60, 0.0, 0.0), Eigen::Vector3d::Zero()); // Translate in x direction
    }

    // Set up data arrays for analysis
    std::vector<std::array<double,3>> desiredConfiguration;  desiredConfiguration.resize(simulationSteps);
    std::vector<std::array<double,3>> actualConfiguration;   actualConfiguration.resize(simulationSteps);
    std::vector<std::array<double,2>> poseError;             poseError.resize(simulationSteps);
    std::vector<std::array<double,2>> controlInputs;         controlInputs.resize(simulationSteps);
    std::vector<std::vector<double>> predictedConfiguration; predictedConfiguration.resize(simulationSteps);
    
    // Run the simulation
    bool track_failure = false;
    for (int i = 0; i < simulationSteps && ! track_failure; ++i)
    {
        double simTime = i / controlFrequency;                                                      // Dividing is more numerically stable
        
        std::vector<RobotLibrary::Model::DifferentialDriveState> desiredStates;                     // Query the desired state from the trajectory across the control horizon
        
        for (int j = 0; j <= predictionSteps; ++j)
        {
            const auto &[pos, vel, acc] = trajectory.query_state(simTime + j / controlFrequency);   // Sample the trajectory across the horizon
         
            // We need to put it in a data structure
            RobotLibrary::Model::DifferentialDriveState state;
            state.pose = RobotLibrary::Model::Pose2D(pos[0], pos[1], pos[2]);
            state.velocity = vel;
            
            desiredStates.push_back(state);   
        }

        try
        {
            controlInput = controller.track_trajectory(desiredStates, obstacles);                   // Solve the predictive control problem
        }
        catch (const std::exception &exception)
        {
            std::cerr <<"[ERROR] [DIFFERENTIAL DRIVE PREDICTIVE CONTROL] "
                                     "Failed to solve trajectory tracking:\n"
                                     << std::string(exception.what());
                                     std::cout<<"\n\n Breaking";
             track_failure = true; 
             desiredConfiguration.resize(i-1);
             actualConfiguration.resize(i-1);
             poseError.resize(i-1);
             controlInputs.resize(i-1);
             
             std::cout<<"\n\n Breaking2";
             break;
        }
        
        // Save data for future analysis
        desiredConfiguration[i] = {desiredStates[0].pose.translation()[0], desiredStates[0].pose.translation()[1], desiredStates[0].pose.angle()};
        actualConfiguration[i]  = {actualPose.translation()[0], actualPose.translation()[1], actualPose.angle()};
        poseError[i]            = {(desiredStates[0].pose.translation() - controller.pose().translation()).norm(), abs(desiredStates[0].pose.angle() - controller.pose().angle())};                   
        controlInputs[i]        = {controlInput[0], controlInput[1]};
        
        for( int j = 0; j < predictionSteps; ++j)
        {
        
        }
        
        // For next loop
        controller.update_state(actualPose, controlInput);
        actualPose = controller.predicted_pose();                                                   // Propagate the state
    }
    
    std::ofstream file;
    std::cout<<"\n Got here before fump";

    std::cout<<"\n got here before dump";
    // Save the trajectory data
    file.open("desired_configuration_data.csv");
    for(int i = 0; i < desiredConfiguration.size(); ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << desiredConfiguration[i][j];
      file << "\n";
    }
    file.close();

    // Save the actual configuration data
    file.open("actual_configuration_data.csv");
    for(int i = 0; i < actualConfiguration.size(); ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << actualConfiguration[i][j];
      file << "\n";
    }
    file.close();  
    
    // Save the control data
    file.open("control_input_data.csv");
    for(int i = 0; i < controlInputs.size(); ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 2; ++j) file << "," << controlInputs[i][j];
      file << "\n";
    }
    file.close(); 
    
    // Save the error data
    file.open("tracking_error_data.csv");
    for (int i = 0; i < poseError.size(); ++i)
    {
        file << (double)(i / controlFrequency);
        for(int j = 0; j < 2; ++j) file << "," << poseError[i][j];
        file << "\n";
    }
    file.close();

    /* NOTE: This needs to be re-worked... indices have changed
    // Save the obstacle*/
    file.open("obstacle_data.csv");
    for(int i = 0; i < obstacles.size(); ++i)
    {
        file << (double)(i);
        file << "," << obstacles[i][0].pose().translation()(0) << "," << obstacles[i][0].pose().translation()(1) << "," << xSemiAxis << "," << ySemiAxis << "\n";
    }
    file.close();
    
    
    std::cout << "[INFO] [DIFFERENTIAL DRIVE PREDICTIVE CONTROL] Numerical simulation complete. "
              << "Data saved to .csv files for analysis.\n";

    return 0;                                                                                       // No problems with main
}
