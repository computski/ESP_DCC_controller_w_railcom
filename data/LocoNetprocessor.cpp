// 
// 
// 

#include "LocoNetprocessor.h"
#include "Global.h"
#include "DCCcore.h"


using namespace nsLOCONETprocessor;

static std::vector<CLIENTMESSAGE> messages;



void nsLOCONETprocessor::handleLocoNet(void* arg, AsyncClient* client, void* data, size_t len) {
	trace(Serial.printf("\nLOCOnet %s \n", client->remoteIP().toString().c_str());)

		//malloc gives more efficient memory usage than a fixed buffer - remember to use free
		char* buffer;
	buffer = (char*)malloc(len + 1);
	//incoming *data was void, need to cast to const char
	strncpy(buffer, (const char*)data, len);
	buffer[len] = '\0';  //terminate with a null

	//just echo for now
	Serial.printf("L: %s\r\n", buffer);


	//use #include <stdlib.h> // For strtol
	//long int decimalValue = strtol(hexString, &endPtr, 16);  &endPtr you can use NULL if not interested
	//but it will be a ptr to char immediately after the hex so can use this to advance along a space separated string of hex
	queueMessage("83 7C\r\n", client);  //loconet power on.  82 7D=off



	free(buffer);
}

void nsLOCONETprocessor::tokenProcessor(char* msg, AsyncClient* client) {}

void nsLOCONETprocessor::buildBroadcastQueue(bool clearQueue) {
	if (clearQueue) {
		messages.clear();
		return;
	}
}

/// <summary>
/// Transmit queued messages to a specific client. This will also transmit any broadcast messages
/// that have been queued up.  It will iterate the message queue.
/// </summary>
/// <param name="client">specific client to send to, cannot be nullptr</param>
void nsLOCONETprocessor::sendToClient(AsyncClient* client) {
	if (messages.empty()) return;
	if (client == nullptr) return;

	std::string outBoundMessage;
	outBoundMessage.clear();

	//build the outBoundMessage from client specific messages plus any broadcast messages
	for (auto m : messages) {
		if ((m.toClient == nullptr) || (m.toClient == client)) {
			outBoundMessage.append(m.msg);
		}
	}

	//find specific client messages or nullptr broadast, and send
	//const char *data = s.c_str(); will cause a crash if you use it to call client->add
	//need to copy the data to a new array

	if (outBoundMessage.size() > 0) {
		char* data = new char[outBoundMessage.size() + 1];
		copy(outBoundMessage.begin(), outBoundMessage.end(), data);
		data[outBoundMessage.size()] = '\0';

		//send over TCP/IP
		if (client->space() > sizeof(data) && client->canSend()) {
			client->add(data, strlen(data));
			client->send();

			Serial.println("send");
		}

		//we used new to create *data.  delete now else you create a memory leak
		delete data;
	}


}
void nsLOCONETprocessor::broadcastSMreadResult(uint16_t cvReg, int16_t cvVal) {}


/// <summary>
/// queue a message for a specific client, or all
/// </summary>
/// <param name="s">standard string containing message</param>
/// <param name="client">spcific client, or all if nullptr</param>
void nsLOCONETprocessor::queueMessage(std::string s, AsyncClient* client) {
	CLIENTMESSAGE m;
	m.toClient = client;
	m.msg = s;
	messages.push_back(m);
}

void nsLOCONETprocessor::setPower(bool powerOn);