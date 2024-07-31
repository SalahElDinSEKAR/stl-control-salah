function [newTrainInput, newTrainOutput, Options] = prepareDataImDAggerFlyingRobot(mergedData, dataStruct, options)
% Prepare data for flying robot with DAgger
%
% Copyright 2019 The MathWorks, Inc.

% Extract the dataStruct variables
numObservations = dataStruct.numObservations;
numActions = dataStruct.numActions;
actionIndex = dataStruct.actionIndex;
stateIndex = dataStruct.stateIndex;

% Calculate number of validation rows
totalRows = size(mergedData,1);
validationSplitPercent = dataStruct.validationSplitPercent;
numValidationDataRows = floor(validationSplitPercent*totalRows);    

% Create validation and training data
randomIdx = randperm(totalRows,numValidationDataRows);
randomData = mergedData(randomIdx,:);
trainDataIdx = setdiff(1:totalRows,randomIdx);

newValidationData = randomData(1:numValidationDataRows,:);
newTrainData = mergedData(trainDataIdx,:);
numTrainDataRows = size(newTrainData,1);

% Prepare new dataset for training
newTrainInput = newTrainData(:,stateIndex);
newTrainOutput = newTrainData(:,actionIndex);

newValidationInput = newValidationData(:,stateIndex);
newValidationOutput = newValidationData(:,actionIndex);
newValidationCellArray = {newValidationInput, newValidationOutput};

% Set new validation cell array in neural network options
Options = options;
Options.ValidationData = newValidationCellArray;

end