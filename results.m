function results(sensor_noise,movement_noise,num_sims)

	%Default to 10 simulations with 10% noise if not enough arguments provided
	if nargin ~= 3
		sensor_noise = 0.1;
		movement_noise = 0.1;
		num_sims = 10;
	end

	%initialise number of successes/kinda successes at 0
	successes = 0;
	kinda_successes = 0;

	%loop for the required number of simulations
	for i=1:num_sims

		%Store outputs from trinity02 function in array
		%   Cycles      |  Travel Dist  |     Goal      |   Obs. Crash  |   Wall Crash  |
		[result_tab(i,1),result_tab(i,2),result_tab(i,3),result_tab(i,4),result_tab(i,5),result_tab(i,6)] = trinity02(sensor_noise,movement_noise);
		clc;

		progress = floor(i/num_sims*100);

		fprintf('=============== SIMULATING: %i%% ================\n',progress);

		%If goal is reached...
		if result_tab(i,3)==1
			%...with no collisions, add 1 to successes
			if result_tab(i,4)==0
				successes = successes+1;
			%...with collisions, add 1 to kinda successes
			else
				kinda_successes = kinda_successes+1;
			end
		end
	end

	clc;
	fprintf('==================== COMPLETE ===================\n',progress);

	%Calculate and display success rate
	success_rate = successes/num_sims;
	success_pc = 100*success_rate;
	kinda_success_rate = kinda_successes/num_sims;
	kinda_success_pc = 100*kinda_success_rate;
	overall_rate = success_rate + kinda_success_rate;
	overall_pc = 100*overall_rate;
	%calculate the average number of computing cycles for the sucessful sims
	ind = find(result_tab(:,3)==1);
	avg_cyc_no_crashes = mean(result_tab(ind,1));
	ind = find(result_tab(:,3)==1 & result_tab(:,4) > 0);
	avg_cyc_with_crashes = mean(result_tab(ind,1));
	avg_cyc_overall = (avg_cyc_with_crashes + avg_cyc_no_crashes)/2;

	%Print final results
	fprintf('========= RESULTS AFTER %i SIMULATIONS =========\n',i);
	fprintf('------------------- No Crashes -------------------\nSuccess Rate: %5.2f%% Average No. of Cycles: %5.2f\n',success_pc,avg_cyc_no_crashes);
	fprintf('------------------ With Crashes ------------------\nSuccess Rate: %5.2f%% Average No. of Cycles: %5.2f\n',kinda_success_pc,avg_cyc_with_crashes);
	fprintf('-------------------- Overall ---------------------\nSuccess Rate: %5.2f%% Average No. of Cycles: %5.2f\n',overall_pc,avg_cyc_overall);
	
end