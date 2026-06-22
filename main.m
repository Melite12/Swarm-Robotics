%% version 1.0.1

%% Legal Disclaimer
% All rights reserved. The "Kalman Filter from the Ground Up" book examples source code
% is sold without warranty, either express or implied. The author will not be held liable for
% any damages caused or alleged to be caused directly or indirectly by this book and/or the
% source code.

%%
exNum = 1;  % set example number

%% change the present working directory to the example directory
myName = mfilename('fullpath');
[myPath, ~, ~] = fileparts(myName);
cd([myPath filesep 'Example_' num2str(exNum)]);               

%% run example
eval(['example_' num2str(exNum)]);

%% revert the present working directory to the main directory
cd(myPath);