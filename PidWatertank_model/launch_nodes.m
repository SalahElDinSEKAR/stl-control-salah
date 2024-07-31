function launch_nodes(H0,ref0)
    % Define the paths to the ROS 2 setup and workspace setup files
    ros2SetupFile = '/opt/ros/humble/setup.bash';
    workspaceSetupFile = '~/Documents/stage/ros2-matlab/install/setup.bash';
    launchFile = '/home/sekars/Documents/stage/ros2-matlab/src/testing/launch/automatisation.py'; % Your launch file within the workspace
    convertionFile= '/home/sekars/Documents/stage/ros2-matlab/bag_to_csv.py';

    % Get the current LD_LIBRARY_PATH
    currentLDLibraryPath = getenv('LD_LIBRARY_PATH');
 
    % Define the path to the system libstdc++.so.6 (you may need to adjust this path)
    systemLibPath = '/usr/lib/x86_64-linux-gnu';
 
    % Prepend the system library path to LD_LIBRARY_PATH
    newLDLibraryPath = [systemLibPath ':' currentLDLibraryPath];
 
    % Set the modified LD_LIBRARY_PATH
    setenv('LD_LIBRARY_PATH', newLDLibraryPath)
    % Construct the command to source the setup files and launch the nodes
    command = sprintf('source %s && source %s', ros2SetupFile, workspaceSetupFile);
    system(command)
    
    command = sprintf('python3 %s %f %f',launchFile,H0,ref0);
    % Use the system command to execute the constructed command
    system(command)

    % turn bags to csv fromat

    command = sprintf('python3 %s',convertionFile);
    system(command)

    % Restore the original LD_LIBRARY_PATH% 
    setenv('LD_LIBRARY_PATH', currentLDLibraryPath);
end
