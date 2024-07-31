% Breach
function init_paths(model_name)
    %% Change to your local path to breach 
    
    breach_path='/home/sekars/Documents/stage/breach';
    %breach_path='/home/alex/workspace/decyphir/breach-dev';

    addpath(breach_path)
    addpath('shared_code')
    InitBreach

    % path of model
    addpath(model_name);