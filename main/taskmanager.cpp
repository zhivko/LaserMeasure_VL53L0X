#include "Taskmanager.h"

void taskmanageTask(void * pvParameters){
	TaskStatus_t *pxTaskStatusArray = nullptr;
	volatile UBaseType_t uxArraySize;
	uint32_t ulTotalRunTime;
	char outputBuffer[256];

	while(true){
		uxArraySize = uxTaskGetNumberOfTasks();

		if(pxTaskStatusArray != nullptr) //Free memory
			delete pxTaskStatusArray;

		pxTaskStatusArray = (TaskStatus_t*) pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
		sprintf(outputBuffer, "%15s%15s%15s%15s%15s%15s%15s%15s\r\n", "NAME", "ID", "STATE", "PRIO", "BASE", "TIME", "CPU", "STACK");
		Serial.print(outputBuffer);
		TaskStatus_t xTaskDetails;

		for(int i = 0; i < uxArraySize; i++){

			vTaskGetInfo( /* The handle of the task being queried. */
							  pxTaskStatusArray[i].xHandle,
			                  /* The TaskStatus_t structure to complete with information
			                  on xTask. */
			                  &xTaskDetails,
			                  /* Include the stack high water mark value in the
			                  TaskStatus_t structure. */
			                  pdTRUE,
			                  /* Include the task state in the TaskStatus_t structure. */
			                  eInvalid );
			uint32_t taskWatermark = xTaskDetails.usStackHighWaterMark;
			//((tskTCB*)pxTaskStatusArray[i].xHandle)

			pxTaskStatusArray[i].

			sprintf(outputBuffer, "%15s", pxTaskStatusArray[i].pcTaskName);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "%15u", pxTaskStatusArray[i].xTaskNumber);
			Serial.print(outputBuffer);



			switch (pxTaskStatusArray[i].eCurrentState){
			case eRunning:
				sprintf(outputBuffer, "%15s", "running");
				Serial.print(outputBuffer);
				break;
			case eReady:
				sprintf(outputBuffer, "%15s", "ready");
				Serial.print(outputBuffer);
				break;
			case eBlocked:
				sprintf(outputBuffer, "%15s", "blocked");
				Serial.print(outputBuffer);
				break;
			case eSuspended:
				sprintf(outputBuffer, "%15s", "suspended");
				Serial.print(outputBuffer);
				break;
			case eDeleted:
				sprintf(outputBuffer, "%15s", "deleted");
				Serial.print(outputBuffer);
				break;
			default:
				sprintf(outputBuffer, "%15s", "unkown");
				Serial.print(outputBuffer);
				break;
			}

			sprintf(outputBuffer, "%15u", pxTaskStatusArray[i].uxCurrentPriority);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "%15u", pxTaskStatusArray[i].uxBasePriority);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "%15u", pxTaskStatusArray[i].ulRunTimeCounter);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "%15f", (float) pxTaskStatusArray[i].ulRunTimeCounter / (float) ulTotalRunTime);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "%15d", pxTaskStatusArray[i].usStackHighWaterMark);
			Serial.print(outputBuffer);
			sprintf(outputBuffer, "\r\n");
			Serial.print(outputBuffer);
		}

		delay(5000);
	}
}
