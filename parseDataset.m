#extracts and parse the data that come from the sensor of a real robot
#filename: path to position of dataset file
#parameters: structure that contains initial parameters
#data: structure that stores extracted data from dataset
function [parameters, data] = parseDataset(filename)
  fid = fopen(filename, 'r');
  if fid == -1
    printf("Cannot open file.");
    return;
  end

  #initialize containers
  time = [];
  steer_ticks = [];
  traction_ticks = [];
  model_x = [];
  model_y = [];
  model_theta = [];
  tracker_x = [];
  tracker_y = [];
  tracker_theta = [];

  #skip the first 9 lines that are comments and get parameters
  for i = 1:9
    line = fgetl(fid);
    if strfind(line,'parameter_values')
      tokens = ostrsplit(line, ' ');
      tokens = tokens(~cellfun(@isempty, tokens));
      parameters.Ksteer = str2double(tokens(1,2));
      parameters.Ktraction = str2double(tokens(1,2));
      parameters.axis_length = str2double(tokens(1,3));
      parameters.steer_offset = str2double(tokens(1,4));
    end
    if strfind(line,'joints_max_enc_values')
      tokens = ostrsplit(line, ' ');
      tokens = tokens(~cellfun(@isempty, tokens));
      parameters.steering = str2double(tokens(1,2));
      parameters.traction_wheel = str2double(tokens(1,3));
    end
  end

  while ~feof(fid)
    line = fgetl(fid);
    if isempty(line) || startsWith(line, "#")
      continue;
    end

    #split string in array to extract values
    tokens = ostrsplit(line, ':');

    #store extracted values
    time(end+1,1) = str2double(erase(char(tokens(1,2)), ' ticks'));

    temp = erase(char(tokens(1,3)), ' model_pose');
    a_temp = ostrsplit(temp, ' ');
    a_temp = a_temp(~cellfun(@isempty, a_temp));
    steer_ticks(end+1,1) = str2double(a_temp(1,1));
    traction_ticks(end+1,1) = str2double(a_temp(1,2));

    temp = erase(char(tokens(1,4)), ' tracker_pose');
    a_temp = ostrsplit(temp, ' ');
    a_temp = a_temp(~cellfun(@isempty, a_temp));
    model_x(end+1,1) = str2double(a_temp(1,1));
    model_y(end+1,1) = str2double(a_temp(1,2));
    model_theta(end+1,1) = str2double(a_temp(1,3));

    temp = char(tokens(1,5));
    a_temp = ostrsplit(temp, ' ');
    a_temp = a_temp(~cellfun(@isempty, a_temp));
    tracker_x(end+1,1) = str2double(a_temp(1,1));
    tracker_y(end+1,1) = str2double(a_temp(1,2));
    tracker_theta(end+1,1) = str2double(a_temp(1,3));
  end

  fclose(fid);

  #store into a struct
  data.time = time;
  data.steer_ticks = steer_ticks;
  data.traction_ticks = traction_ticks;
  data.model_pose = [model_x model_y model_theta];
  data.tracker_pose = [tracker_x tracker_y tracker_theta];
end

