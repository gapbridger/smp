#include "../inc/module.h"
#include "../inc/smp.h"

// single joint move position sequence
void MovePositionSequence(std::vector<int> module_idx, std::vector<std::vector<float>>& pos, SMP* smp); // we must use pointer here...

void main()
{
	// simple test function of moving module 5 to 0 position
	const int num_module = kModuleNumber;
	int num_joints = 3;
	int path_len = 5;
	std::vector<std::vector<float>> pos_sequence(num_joints, std::vector<float>(path_len));
	std::vector<int> module_idx(num_joints);
	module_idx[0] = 0; module_idx[1] = 3; module_idx[2] = 5;
	for(int i = 0; i < path_len; i++)	
	{
		pos_sequence[0][i] = i % 2 == 0 ? -5 : 5;
		pos_sequence[1][i] = i % 2 == 0 ? 40 : 50;
		pos_sequence[2][i] = i % 2 == 0 ? 70 : 80;
		// pos_sequence[2][i] = i % 2 == 0 ? 70 : 80;
	}
	SMP smp;
	smp.StartCANBusComm();
	for(int module_id = 0; module_id < num_module; module_id++)
		smp.Acknowledge(module_id); // acknowledge, quit error
	for(int i = 0; i < num_joints; i++)
		smp.GetState(module_idx[i], 0.04, 0x07);

	smp.CANPollingStart();
	// smp.MovePosSequenceStart();	
	boost::thread thread(boost::bind(&MovePositionSequence, module_idx, pos_sequence, &smp));
	/*smp.MovePosition(0, 0, 5.0, 20.0, 9.99);
	smp.MovePosition(3, 45, 5.0, 20.0, 6.0);
	smp.MovePosition(5, 75, 5.0, 20.0, 4.0);*/
	thread.join();
	smp.CANPollingStop();	
	
	system("pause");

	
	
}


void MovePositionSequence(std::vector<int> module_idx, std::vector<std::vector<float>>& pos, SMP* smp)
{
	boost::timer::cpu_timer timer;	
	boost::timer::nanosecond_type const status_update_interval(80000000LL); // 2ms interval	
	std::cout << "status query thread started..." << std::endl;
	boost::timer::cpu_times elapsed_times(timer.elapsed());
	boost::timer::nanosecond_type elapsed(elapsed_times.wall);

	int path_len = pos[0].size();
	int num_joints = pos.size();
	std::vector<int> count(num_joints);
	for(int i = 0; i < num_joints; i++)
		count[i] = 0;
	// keep querying
	std::cout << "entering motion sequence thread..." << std::endl;
	std::cout << "path length: " << path_len << std::endl;
	std::vector<float> vel(num_joints);
	std::vector<float> acc(num_joints);
	std::vector<float> curr(num_joints);
	for(int i = 0; i < num_joints; i++)
	{
		vel[i] = 5.0;
		acc[i] = 20.0;
	}
	curr[0] = 9.99; curr[1] = 6.00; curr[2] = 4.00;
	while(1)
	{
		for(int i = 0; i < num_joints; i++)
		{
			if(smp->PositionReached(module_idx[i]) && count[i] < path_len)
			{
				smp->MovePosition(module_idx[i], pos[i][count[i]], vel[i], acc[i], curr[i]);
				std::cout << "module: " << i << " count: " << count[i] << "..." << std::endl;
				++count[i];
			}
			else if(smp->PositionReached(module_idx[i]))
			{
				++count[i];
			}
		}
		timer.stop();
		timer.start();
		elapsed = timer.elapsed().wall;	
		while (elapsed <= status_update_interval)		
			elapsed = timer.elapsed().wall;		

		int return_flag = 1;
		for(int i = 0; i < num_joints; i++)
		{
			if(count[i] <= path_len)
			{
				return_flag = 0;
				break;
			}
		}
		if(return_flag)
			break;
	}	
}
