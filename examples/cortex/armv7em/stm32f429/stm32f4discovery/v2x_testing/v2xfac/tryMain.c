/*
 * tryMain.c
 *
 *  Created on: Feb 5, 2022
 *      Author: Sayed
 */

#include "./inc/asn1crt_encoding_uper.h"
#include "./inc/CAM.h"
#include "./inc/DENM.h"

void fill_CAM(V2xFac_CamMessageRootType* message)
{
	message->itsPduHeader.protocolVersion = 1;
	message->itsPduHeader.messageId = 2;
	message->itsPduHeader.stationId = 124567;
	message->coopAwareness.generationDeltaTime = 11409;
	message->coopAwareness.camParameters.basicContainer.stationType = 10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.altitude.altitudeValue=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.latitude=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.longitude=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.posConfidenceEllipse.semiMajorConfidence=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.posConfidenceEllipse.semiMajorOrientation=10;
	message->coopAwareness.camParameters.basicContainer.referencePosition.posConfidenceEllipse.semiMinorConfidence=10;
	message->coopAwareness.camParameters.highFrequencyContainer.kind = basicVehicleContainerHighFrequency_PRESENT;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.accelerationControl.arr[0] = 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneID = 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLatitude = 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLongitude = 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.cenDsrcTollingZone.exist.cenDsrcTollingZoneID = 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvatureCalculationMode= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.driveDirection= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.accelerationControl= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.cenDsrcTollingZone= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.lanePosition= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.lateralAcceleration= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.performanceClass= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.steeringWheelAngle= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.exist.verticalAcceleration= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.lanePosition= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.performanceClass= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleWidth= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationValue= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence= 1;
	message->coopAwareness.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateValue= 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.kind= basicVehicleContainerLowFrequency_PRESENT;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.exteriorLights.arr[0] = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.arr[0].pathDeltaTime = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.arr[0].pathPosition.deltaAltitude = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.arr[0].pathPosition.deltaLatitude = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.arr[0].pathPosition.deltaLongitude = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.arr[0].exist.pathDeltaTime = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.nCount = 1;
	message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.vehicleRole = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.kind = emergencyContainer_PRESENT;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.emergencyPriority.arr[0] = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.incidentIndication.causeCode = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.incidentIndication.subCauseCode = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.lightBarSirenInUse.arr[0] = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.exist.emergencyPriority = 1;
	message->coopAwareness.camParameters.specialVehicleContainer.u.emergencyContainer.exist.incidentIndication = 1;
	message->coopAwareness.camParameters.exist.lowFrequencyContainer = 1;
	message->coopAwareness.camParameters.exist.specialVehicleContainer = 1;
}

//void printCAM(V2xFac_CamMessageRootType* message)
//{
//	printf("protocolVersion = %d\n",message->itsPduHeader.protocolVersion);
//	printf("messageId = %d\n",message->itsPduHeader.messageId);
//	printf("stationId = %d\n",message->itsPduHeader.stationId);
//	printf("generationDeltaTime = %d\n",message->coopAwareness.generationDeltaTime);
//	printf("stationType = %d\n",message->coopAwareness.camParameters.basicContainer.stationType);
//	printf("altitudeConfidence = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence);
//	printf("altitudeValue = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.altitude.altitudeValue);
//	printf("latitude = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.latitude);
//	printf("longitude = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.longitude);
//	printf("semiMajorConfidence = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.posConfidenceEllipse.semiMajorConfidence);
//	printf("semiMajorOrientation = %d\n",message->coopAwareness.camParameters.basicContainer.referencePosition.posConfidenceEllipse.semiMajorOrientation);
//	printf("...\n");
//	printf("lowFrequencyContainer = %d\n",message->coopAwareness.camParameters.exist.lowFrequencyContainer);
//	printf("specialVehicleContainer = %d\n",message->coopAwareness.camParameters.exist.specialVehicleContainer);
//}

int main(void)
{

	V2xFac_DenmMessageRootType DENMessage;

	/* Created CAM Message to be encodded. */
	V2xFac_CamMessageRootType message;

	/* CAM Message to store the decoding output. */
	V2xFac_CamMessageRootType decoddedMessage;

	/* boolean Flag to check weather the encoding/decoding operation is done successfuly or not. */
	flag result;

	/* Encoding buffer to hold the encodded bytes, it is an array of size needed for the whole CAM (676 bytes) + 1. */
	static byte encBuff[V2xFac_CamMessageRootType_REQUIRED_BYTES_FOR_ENCODING + 1]; /* +1 for zerosized types */

	/* Bit Stream to hold all data about the encodded message:
	 * byte* buf; 				--> pointer to the encodded buffer.
	   long count; 				--> Size of buffer in bytes (676).
	   long currentByte;		--> Size of the encodded values (last byte index in the encodded message).
		 Next available bit for writting.
		Possible vallues 0..7, 0 is most significant
		bit of current byte
		int currentBit;			--> Index of the last bit in the last byte in the encodded message.
		//PushDataFnc pushData;
		void* pushDataPrm;
		//FetchDataFnc fetchData;
		void* fetchDataPrm;
	*/
	BitStream encoddedMessage;

	/* Integer to hold the error code (in case of error in encoding/decoding). */
	int pErrCode;

	/* Fill the CAM message to be encodded. */
	fill_CAM(&message);

	/* Initialization of all elements in the BitStream structure and it takes the following parameters:
	 * 1- pointer to the bitStream structure to be initialized.
	 * 2- pointer to the buffer which will hold the encodded bytes.
	 * 3- Size of the buffer. */
	BitStream_Init(&encoddedMessage, encBuff, V2xFac_CamMessageRootType_REQUIRED_BYTES_FOR_ENCODING);

	/* Encoding Function for CAM message, it takes the following parameters:
	 * 1- Pointer to the message to be encodde.
	 * 2- Pointer to BitStream to hold all data about the encodded buffer.
	 * 3- Pointer to the error code holder to store the error code if it happens
	 * 4- Boolean value to validate the message before encoding it or not. */
	result = V2xFac_CamMessageRootType_Encode(&message, &encoddedMessage, &pErrCode, TRUE);

	/* Checking if encoding is done correctly. */
	if(result)
	{
		result =0;
		/* Printing the encoding message and the size of the encoded message. */
		//printf("Encodding is Done\t%d\n", encoddedMessage.currentByte);

		/* For Loop to print the whole encodded bytes. */
//		for(int i=0; i<encoddedMessage.currentByte; i++)
//		{
//			printf("%02X", *(encoddedMessage.buf +i));
//		}
//		printf("\n\n");

		/* Function to refresh the bitstream (it must be used before using decoding function). */
		BitStream_AttachBuffer(&encoddedMessage, encBuff,  V2xFac_CamMessageRootType_REQUIRED_BYTES_FOR_ENCODING);

		/* Decoding Function for CAM message, it takes the following parameters:
		 * 1- Pointer to the message to hold the decodded data..
		 * 2- Pointer to BitStream which hold all data about the encodded buffer.
		 * 3- Pointer to the error code holder to store the error code if it happens */
		result = V2xFac_CamMessageRootType_Decode(&decoddedMessage, &encoddedMessage, &pErrCode);

		/* Checking if the decoding is done correctly. */
//		if(result)
//		{
//			printf("Decoding is Done\n");
//			printCAM(&decoddedMessage);
//		}
//		else
//			printf("Decoding is Failed");
		result =0;

		if (message.itsPduHeader.protocolVersion == decoddedMessage.itsPduHeader.protocolVersion)
		{
			if(message.coopAwareness.camParameters.exist.specialVehicleContainer == decoddedMessage.coopAwareness.camParameters.exist.specialVehicleContainer)
				result =1;
		}

	}
//	else
//		printf("Encoding is Failed\n");

	return 0;
}
