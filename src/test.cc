#include "../inc/module.h"
#include "../inc/smp.h"

void main()
{
	// simple test function of moving module 5 to 0 position
	const int num_module = kModuleNumber;
	int target_module_id = 5;
	SMP smp;
	smp.StartCANBusComm();
	for(int module_id = 0; module_id < num_module; module_id++)
		smp.Acknowledge(module_id); // acknowledge, quit error
	smp.GetState(target_module_id, 0.04, 0x07);
	smp.MovePosition(target_module_id, 0, 4, 20, 4.0);
}