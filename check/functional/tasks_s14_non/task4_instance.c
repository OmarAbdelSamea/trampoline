/*Instance of task t4*/

//#include "test_instances.h"
#include "embUnit.h"
#include "tpl_os.h"

DeclareEvent(Task4_Event1);
DeclareEvent(Task4_Event2);
DeclareEvent(Task4_Event3);
DeclareEvent(Task4_Event4);
DeclareEvent(Task4_Event5);
DeclareEvent(Task4_Event6);
DeclareEvent(Task4_Event7);
DeclareEvent(Task4_Event8);

/*test case:test the reaction of the system called with 
an activation of a task*/
static void test_t4_instance(void)
{
	int result_inst_1, result_inst_2, result_inst_3, result_inst_4, result_inst_5, result_inst_6, result_inst_7, result_inst_8, result_inst_9;

	result_inst_1 = ClearEvent(Task4_Event1);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_1); 
	
	result_inst_2 = ClearEvent(Task4_Event2);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_2);
	
	result_inst_3 = ClearEvent(Task4_Event3);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_3); 
	
	result_inst_4 = ClearEvent(Task4_Event4);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_4);
	
	result_inst_5 = ClearEvent(Task4_Event5);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_5); 
	
	result_inst_6 = ClearEvent(Task4_Event6);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_6);
	
	result_inst_7 = ClearEvent(Task4_Event7);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_7); 
	
	result_inst_8 = ClearEvent(Task4_Event8);
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_8);
	
	result_inst_9 = TerminateTask();
	TEST_ASSERT_EQUAL_INT(E_OK , result_inst_9);
}

/*create the test suite with all the test cases*/
TestRef TaskManagementTest_seq14_t4_instance(void)
{
	EMB_UNIT_TESTFIXTURES(fixtures) {
		new_TestFixture("test_t2_instance",test_t4_instance)
	};
	EMB_UNIT_TESTCALLER(TaskManagementTest,"TaskManagementTest_sequence14",NULL,NULL,fixtures);

	return (TestRef)&TaskManagementTest;
}