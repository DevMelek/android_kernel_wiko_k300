#include <teei_client_api.h>
#include <gp_client.h>

unsigned int TEEIC_InitializeContext(const char *name, struct TEEIC_Context *context)
{
	return TEEC_InitializeContext(name, context);
}

void TEEIC_FinalizeContext(struct TEEIC_Context *context)
{
	TEEC_FinalizeContext(context);
}

unsigned int TEEIC_RegisterSharedMemory(struct TEEIC_Context *context, struct TEEIC_SharedMemory *sharedMem)
{
	return TEEC_RegisterSharedMemory(context, sharedMem);
}

unsigned int TEEIC_AllocateSharedMemory(struct TEEIC_Context *context, struct TEEIC_SharedMemory *sharedMem)
{
	return TEEC_AllocateSharedMemory(context, sharedMem);
}

void TEEIC_ReleaseSharedMemory(struct TEEIC_SharedMemory *sharedMem)
{
	TEEC_ReleaseSharedMemory(sharedMem);
}

unsigned int TEEIC_OpenSession(struct TEEIC_Context *context,
				struct TEEIC_Session *session,
				const TEEIC_UUID *destination,
				unsigned int connectionMethod,
				const void *connectionData,
				struct TEEIC_Operation *operation,
				unsigned int *returnOrigin)
{
	return TEEC_OpenSession(context, session, destination, connectionMethod,
				connectionData, operation, returnOrigin);
}


void TEEIC_CloseSession(struct TEEIC_Session *session)
{
	TEEC_CloseSession(session);
}

unsigned int TEEIC_InvokeCommand(struct TEEIC_Session *session,
				unsigned int commandID,
				struct TEEIC_Operation *operation,
				unsigned int *returnOrigin)
{
	return TEEC_InvokeCommand(session, commandID, operation, returnOrigin);
}


void TEEIC_RequestCancellation(struct TEEIC_Operation *operation)
{
	TEEC_RequestCancellation(operation);
}



