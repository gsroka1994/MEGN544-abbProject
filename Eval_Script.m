%% Evaluation Script
function Eval_Script(projectNumber,functionName,N)
if ~exist('functionName','var')
    functionName = {'all'};
end

load('Test_Cases_2.mat');

fun_list{1} = {'rotX';'rotY';'rotZ';'rpy2Rot';'rot2RPY';'cpMap';'angleAxis2Rot';'rot2AngleAxis';'rot2Quat';'quat2Rot';'twist2Transform';'transform2Twist';'dhTransform';};
%fun_list{2} = {'createLink';'dhFwdKine';'invKineWidowXArm'};
fun_list{2} = {'createLink';'dhFwdKine';'abbInvKine';'constAccelInterp'};
fun_list{3} = {'velocityJacobian';'rotationError';'transError';'dhInvKine'};
fun_list{4} = {'newtonEuler'};
switch projectNumber
    case 1
        function_list = fun_list{1};
    case 2
        function_list = fun_list{2};
    case 3
        function_list = fun_list{3};
    case 4
        function_list = fun_list{4};
end

results = struct();

N_times = 1;
x = zeros(N_times,1);for c=1:N_times, x(c) = timeit(@()magic(501),1); end
baseLineTime = mean(x);
% for j=1:size(functionName,1)
j=1;
fun_count = 1;
for i=1:length(test_case)
    
    
    fun = test_case(i);
    
    if ~any(strcmp(fun.Function_Name, function_list))
        continue;
    end
    
    if ~strcmp(functionName(j,:), fun.Function_Name) && ~strcmp(functionName(j,:),'all')
        continue;
    end
    
    if exist('N','var') && (fun_count == N)
        fun_count = fun_count+1;
    elseif exist('N','var') && (fun_count > N)
        break;
    elseif exist('N','var')
        fun_count = fun_count+1;
        continue;
    end
    
    try
        fileID = fopen('tmp.m','w+');
    catch
        fileID = fopen('tmp.m','w+');
    end
    fprintf(fileID,'function [output] = tmp(inputStruct)\n\n');
    
    fprintf(fileID,'score = @(val) 1-1/(1+exp(-(val-.05)*100));\n');
    
    fprintf(fileID,'output = struct(''error_text'',{},''help_score'',0,''effort'',0,''output_error'',[],''timing_performance'',[]);\n\n');
    fprintf(fileID,'output(1).percentError = -1;\n');
    
    fprintf(fileID,'if ~exist(''%s.m'',''file'')\n',fun.Function_Name);
    fprintf(fileID,'\toutput.error_text{end+1} = ''Function %s cannot be found.'';\n',fun.Function_Name);
    fprintf(fileID,'\treturn;\n');
    fprintf(fileID,'end\n');
    
    
    fprintf(fileID,'[~,name,~] = fileparts(which(''%s.m''));\n', fun.Function_Name);
    fprintf(fileID,'caseMatch = strcmp(name,''%s'');\n',fun.Function_Name);
    fprintf(fileID,'if ~caseMatch\n\toutput.error_text{end+1} = [''Case sensitive match for %s does not exist, you used '' name]\n',fun.Function_Name);
    fprintf(fileID,'\treturn;\n');
    fprintf(fileID,'end\n');
    
    
    fprintf(fileID,'T = evalc(''help %s.m'');\n\n',fun.Function_Name);
    fprintf(fileID,['output(1).help_score = 0;\nif strcmp(''No help found for %s.m.'',strtrim(deblank(T)));\n',...
        '\toutput.error_text{end+1} = ''No helpfile Provided.'';\n',...
        'else\n',...
        '\tif numel(T) < %i\n',...
        '\t\toutput.error_text{end+1} = ''Help File Data Insufficient.'';\n',...
        '\t\toutput.help_score = score(max([%i-numel(T),0])/5000);\n',...
        '\telse\n',...
        '\t\toutput.help_score = 1;\n',...
        '\tend\n',...
        'end\n\n'],fun.Function_Name,fun.Help_Length,fun.Help_Length);
    
    fprintf(fileID,'fileID = fopen(''%s.m'');\n',fun.Function_Name);
    fprintf(fileID,'numChar = numel(fread(fileID,''char''))-numel(T);\n');
    fprintf(fileID,'fclose(fileID);\n');
    fprintf(fileID,'output.effort = numChar > %i;\n\n\n',fun.Function_Length);
    
    fprintf(fileID,'try\n');
    fprintf(fileID,'     [');
    for i=1:fun.Number_Outputs
        if( i < fun.Number_Outputs )
            fprintf(fileID,'O%i, ',i);
        else
            fprintf(fileID,'O%i',i);
        end
    end
    fprintf(fileID,'] = %s(',fun.Function_Name);
    
    for i=1:fun.Number_Inputs
        fprintf(fileID,'inputStruct.Inputs.I%i',i);
        if( i < fun.Number_Inputs)
            fprintf(fileID,', ',i);
        else
            fprintf(fileID,');',i);
        end
    end
    
    fprintf(fileID,'\n\n\toutput.percentError = zeros(%i,1);\n',fun.Number_Outputs);
    fprintf(fileID,'\n\n\tuseAlt = false;\n');
    for i=1:fun.Number_Outputs
        fprintf(fileID,'\tif all(size(O%i)==size(inputStruct.Outputs.O%i))\n',i,i);
        fprintf(fileID,'\t\tif isstruct(inputStruct.Outputs.O%i)\n\t\t\tname_list = fieldnames(inputStruct.Outputs.O%i);\n\t\t\tname_check = isfield(O%i,name_list);\n\t\t\tfor i=1:length(name_check)\n\t\t\t\tif ~name_check(i)\n\t\t\t\t\toutput.error_text{end+1} = [name_list{i} '' is missing in struct''];\n\t\t\t\tend\n\t\t\tend\n\t\t\toutput.percentError = 1-sum(name_check)/length(name_check);\n\n\t\t\tif all(name_check)\n\t\t\t\tfor i=1:length(name_check)\n\t\t\t\t\n\t\t\t\t\tref = getfield(inputStruct.Outputs.O%i,name_list{i});\n\t\t\t\t\tval = getfield(O%i,name_list{i});\n\t\t\t\t\tpe = 0;\n\t\t\t\t\tif isempty(ref)\n\t\t\t\t\t\tpe = ~isempty(val)*100;\n\t\t\t\t\telseif any(isnumeric(ref) & abs(ref) > 0)\n\t\t\t\t\t\tpe = 100*norm(val-ref)/norm(ref);\n\t\t\t\t\telseif islogical(ref)\n\t\t\t\t\t\tpe = (ref ~= val)*100;\n\t\t\t\t\telse\n\t\t\t\t\t\tpe = 100*norm(val-ref);\n\t\t\t\t\tend\n\t\t\t\t\toutput.percentError(%i) = max(output.percentError(%i),pe);\n\t\t\t\t\tif pe > 1e-3\n\t\t\t\t\t\toutput.error_text{end+1} = [name_list{i} '' is incorrect. Percent error: '' num2str(pe,2) ''%%''];\n\t\t\t\t\tend\n\t\t\t\tend\n\t\t\tend\n',i,i,i,i,i,i,i);
        
        fprintf(fileID,'\t\telseif norm(inputStruct.Outputs.O%i) >0\n',i);
        if i==1
            fprintf(fileID,'\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%i)/norm(inputStruct.Outputs.O%i);\n',i,i,i,i);
            fprintf(fileID,'\t\t\tif isfield(inputStruct.Outputs,''O%iAlt'')\n',i);
            fprintf(fileID,'\t\t\t\te_alt = 100*norm(O%i-inputStruct.Outputs.O%iAlt)/norm(inputStruct.Outputs.O%iAlt);\n',i,i,i);
            fprintf(fileID,'\t\t\t\tif all(e_alt<output.percentError(%i))\n',i);
            fprintf(fileID,'\t\t\t\t\toutput.percentError(%i) = e_alt;\n',i);
            fprintf(fileID,'\t\t\t\t\tuseAlt = true;\n');
            fprintf(fileID,'\t\t\t\tend\n');
            fprintf(fileID,'\t\t\tend\n');
            fprintf(fileID,'\t\telse\n');
            fprintf(fileID,'\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%i);\n',i,i,i);
            fprintf(fileID,'\t\t\tif isfield(inputStruct.Outputs,''O%iAlt'')\n',i);
            fprintf(fileID,'\t\t\t\te_alt = 100*norm(O%i-inputStruct.Outputs.O%iAlt)/norm(inputStruct.Outputs.O%iAlt);\n',i,i,i);
            fprintf(fileID,'\t\t\t\tif all(e_alt<output.percentError(%i))\n',i);
            fprintf(fileID,'\t\t\t\t\toutput.percentError(%i) = e_alt;\n',i);
            fprintf(fileID,'\t\t\t\t\tuseAlt = true;\n');
            fprintf(fileID,'\t\t\t\tend\n');
            fprintf(fileID,'\t\t\tend\n');
            fprintf(fileID,'\t\tend\n');
        else
            fprintf(fileID,'\t\t\tif ~useAlt\n');
            fprintf(fileID,'\t\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%i)/norm(inputStruct.Outputs.O%i);\n',i,i,i,i);
            fprintf(fileID,'\t\t\telse\n');
            fprintf(fileID,'\t\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%iAlt)/norm(inputStruct.Outputs.O%iAlt);\n',i,i,i,i);
            fprintf(fileID,'\t\t\tend\n');
            fprintf(fileID,'\t\telse\n');
            fprintf(fileID,'\t\t\tif ~useAlt\n');
            fprintf(fileID,'\t\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%i);\n',i,i,i);
            fprintf(fileID,'\t\t\telse\n');
            fprintf(fileID,'\t\t\t\toutput.percentError(%i) = 100*norm(O%i-inputStruct.Outputs.O%iAlt);\n',i,i,i);
            fprintf(fileID,'\t\t\tend\n');
            fprintf(fileID,'\t\tend\n');
        end
        fprintf(fileID,'\telse\n\t\toutput.percentError(%i) = -1;\n\t\toutput.error_text{end+1} = [''Output Number %i has the wrong size is: ['' int2str(size(O%i)) ''] shoudl be ['' int2str(size(inputStruct.Outputs.O%i)) '']''];\n',i,i,i,i);
        fprintf(fileID,'\tend\n\n');
        
        
    end
    
    %% Run Timing checks
    fprintf(fileID,'\n\twarning(''off'',''all'');\n');
    %fprintf(fileID,'\tN_times=30;\n\tx = zeros(N_times,1);for c=1:N_times, x(c) = timeit(@() magic(501),1); end\n\tbaseLineTime = mean(x);\n');
    %fprintf(fileID,'\tN_times=30;\n\tx = zeros(N_times,1);');
    for k=1:fun.Number_Inputs
        fprintf(fileID,'\n\tI%i = inputStruct.Inputs.I%i;',k,k);
    end
    
    fprintf(fileID,['\n\tN_times=' num2str(N_times) '; x = zeros(N_times,1);for c=1:length(x), x(c) = timeit(@()%s('],fun.Function_Name);
    
    for k=1:fun.Number_Inputs
        fprintf(fileID,'I%i',k);
        if( k < fun.Number_Inputs)
            fprintf(fileID,', ');
        else
            fprintf(fileID,')');
        end
    end
    %fprintf(fileID,',%i);end\n\ttimingRatio=mean(x)/baseLineTime;\n',fun.Number_Outputs);
    %fprintf(fileID,'\toutput.timing_performance = 100*(timingRatio-inputStruct.timingRatio)/inputStruct.timingRatio;\n\n');
    fprintf(fileID,',%i);end\n',fun.Number_Outputs);
    fprintf(fileID,'\toutput.timing_performance = mean(x);');%100*(timingRatio-inputStruct.timingRatio)/inputStruct.timingRatio;\n\n');
    fprintf(fileID,'\n\toutput.timing_performance_stdev = std(x);');%100*(timingRatio-inputStruct.timingRatio)/inputStruct.timingRatio;\n\n');
    fprintf(fileID,'\n\twarning(''on'',''all'');\n');
    
    %%
    fprintf(fileID,'\ncatch ME\n');
    %fprintf(fileID,'\tif strcmp(''MATLAB:dispatcher:InexactCaseMatch'',ME.identifier)\n');
    %fprintf(fileID,'\t\toutput.error_text{end+1} = [''Cannot find an exact (case-sensitive) match for %s ''];\n',fun.Function_Name);
    %fprintf(fileID,'\telse\n');
    fprintf(fileID,'\toutput.error_text{end+1} = [ME.message, ''  Line '' , int2str(ME.stack(1).line) , '' in '' ME.stack(1).name];\n');
    %fprintf(fileID,'\tend\n');
    fprintf(fileID,'end\n');
    
    fprintf(fileID,'\nend\n\n');
    
    % fprintf(fileID,'function runTiming(inputStruct)\n');
    % fprintf(fileID,'for i = 1:100\n');
    % fprintf(fileID,'\t%s(',fun.Function_Name);
    %
    % for k=1:fun.Number_Inputs
    %     fprintf(fileID,'inputStruct.Inputs.I%i',k);
    %     if( k < fun.Number_Inputs)
    %         fprintf(fileID,', ');
    %     else
    %         fprintf(fileID,')');
    %     end
    % end
    % fprintf(fileID,';\nend\n');
    %fprintf(fileID,'end');
    
    
    fclose('all');
    
    a=which('tmp.m');
    [~,output] = evalc('tmp(fun)');
    %%
    if isfield(results,fun.Function_Name)
        fun_results = getfield(results,fun.Function_Name);
    else
        fun_results = struct('Help_Score',[],'Effort_Score',[],'Percent_Error',[],'Error_Msg',{},'Special_Case',[]);
    end
    %         disp(['Function: ' fun.Function_Name])
    %         if length(fun.Special_Case) > 0
    %             disp(['Special Case: ' fun.Special_Case]);
    %         end
    %         fprintf('\tHelp File Score: %1.1f out of 1\n', output.help_score);
    %         fprintf('\tEffort Score: %1.1f out of 1\n', output.effort);
    %         for k = 1:length(output.percentError)
    %             fprintf('\tOutput%i Percent Error: %3.2f%%\n', k, output.percentError(k));
    %         end
    fun_results(end+1).Help_Score = output.help_score;
    fun_results(end).Effort_Score = output.effort;
    fun_results(end).Percent_Error = output.percentError;
    fun_results(end).Error_Msg = output.error_text;
    fun_results(end).Special_Case = fun.Special_Case;
    fun_results(end).Timing_Performance = (output.timing_performance/baseLineTime-fun.timingRatio)/(fun.timingRatio+3*fun.timingRatio_stdev);
    results = setfield(results,fun.Function_Name,fun_results);
    %disp( output.timing_performance);
    % fprintf('\tTiming Performance: ');
    % if abs(output.timing_performance) <= 50
    %     fprintf('Same as baseline\n');
    % elseif output.timing_performance < -50
    %     fprintf('Faster than Baseline (nice!)\n');
    % elseif output.timing_performance > 400
    %     fprintf('Too Slow! Work on your effiency!\n');
    % elseif output.timing_performance > 300
    %     fprintf('Slower than baseline, improvements can be made...\n');
    % else
    %     fprintf('Same as baseline\n');
    % end
    %         fprintf('\tErrors:\t');
    %         if length(output.error_text)
    %             for j=1:length(output.error_text)
    %                 fprintf('\n\t\t%s', (output.error_text{j}));
    %             end
    %             fprintf('\n');
    %         else
    %             fprintf('\tNo Matlab Errors or Warnings\n');
    %         end
    delete('tmp.m')
end
% end

%update for error tolerace > threshold
name_list = fieldnames(results);
for i = 1:length(name_list)
    
    disp(['Function: ' name_list{i}])
    fun_results = getfield(results,name_list{i});
    if ~isempty(fun_results(1).Help_Score)
        fprintf('\tHelp File Score: %1.1f out of 1\n', fun_results(1).Help_Score);
    else
        fprintf('\tHelp File Score: 0 out of 1\n');
    end
    
    if ~isempty(fun_results(1).Effort_Score)
        fprintf('\tEffort Score: %1.1f out of 1\n', fun_results(1).Effort_Score);
    else
        fprintf('\tEffort Score: 0 out of 1\n');
    end
    
    if ~isempty(fun_results(1).Timing_Performance)
        fprintf('\tTiming Performance: ');
        if mean([fun_results(:).Timing_Performance]) < 0.5
            fprintf('Good.\n');
        elseif mean([fun_results(:).Timing_Performance]) < 5
            fprintf('Slow.\n');
        else
            fprintf('Very Slow.\n');
        end
    else
        fprintf('\tEfficiency Score: N/A\n');
    end
    
    for j=1:length(fun_results)
        has_errors = ~isempty(fun_results(j).Error_Msg);
        if has_errors
            break;
        end
    end
    
    if ~has_errors && ~any(any(abs(vertcat(fun_results(:).Percent_Error))>sqrt(eps)))
        fprintf('\tPercent Error All Trials: 0%%. Nice Work!\n');
    else
        err_msg = [];
        for j = 1:length(fun_results)
            if ~isempty(fun_results(j).Error_Msg) && ~strcmp(err_msg,fun_results(j).Error_Msg{1})
                fprintf(['\tTest Case ' int2str(j) ]);
                if length(fun_results(j).Special_Case)
                    fprintf(' Special Case: %s ',fun_results(j).Special_Case);
                end
                fprintf(' Error Message: \n');
                for k=1:length(fun_results(j).Error_Msg)
                    fprintf('\t\t%s\n',fun_results(j).Error_Msg{k});
                    err_msg = fun_results(j).Error_Msg{1};
                end
            end
            
            if (isempty(fun_results(j).Error_Msg) || strcmp(fun_results(j).Error_Msg{1},'Help File Data May Be Insufficient.') || strcmp(fun_results(j).Error_Msg{1},'No helpfile Provided.')) && ~any(any(abs(vertcat(fun_results(:).Percent_Error))>sqrt(eps)))
                fprintf('\tPercent Error All Trials: 0%%.\n');
                break;
            elseif isempty(fun_results(j).Error_Msg) || strcmp(fun_results(j).Error_Msg{1},'Help File Data May Be Insufficient.')|| strcmp(fun_results(j).Error_Msg{1},'No helpfile Provided.')
                if any(any(abs(fun_results(j).Percent_Error(:))>sqrt(eps)))
                    fprintf(['\tTest Case ' int2str(j) ]);
                    if length(fun_results(j).Special_Case)
                        fprintf(' Special Case: %s ',fun_results(j).Special_Case);
                    end
                    if length(fun_results(j).Percent_Error)>1
                        fprintf('\n');
                    end
                    for k=1:length(fun_results(j).Percent_Error)
                        fprintf(['\t\tOutput %i Percent Error: %3.2f%%\n'],k, fun_results(j).Percent_Error(k));
                    end
                end
            else
                break;
            end
        end
        
    end
    disp(' ');
end

