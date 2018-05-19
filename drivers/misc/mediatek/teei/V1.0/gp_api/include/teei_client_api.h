#ifndef __TEEI_CLIENT_API_H_
#define __TEEI_CLIENT_API_H_
#include "tee_client_api.h"
#define TEEIC_PARAM_TYPES(param0Type, param1Type, param2Type, param3Type) \
	(param3Type << 12 | param2Type << 8 | param1Type << 4 | param0Type)


unsigned int TEEIC_InitializeContext(const char *name, struct TEEIC_Context *context);



void TEEIC_FinalizeContext(struct TEEIC_Context *context);

unsigned int TEEIC_RegisterSharedMemory(struct TEEIC_Context *context, struct TEEIC_SharedMemory *sharedMem);


unsigned int TEEIC_AllocateSharedMemory(struct TEEIC_Context *context, struct TEEIC_SharedMemory *sharedMem);


void TEEIC_ReleaseSharedMemory(struct TEEIC_SharedMemory *sharedMem);

unsigned int TEEIC_OpenSession(struct TEEIC_Context *context,
				struct TEEIC_Session *session,
				const struct TEEIC_UUID *destination,
				uint32_t connectionMethod,
				const void *connectionData,
				struct TEEIC_Operation *operation,
				uint32_t *returnOrigin);

void TEEIC_CloseSession(struct TEEIC_Session *session);

unsigned int TEEIC_InvokeCommand(struct TEEIC_Session *session,
				uint32_t commandID,
				struct TEEIC_Operation *operation,
				uint32_t *returnOrigin);


void TEEIC_RequestCancellation(struct TEEIC_Operation *operation);

void allow(struct TEEC_Operation *operation);
void block(struct TEEC_Operation *operation);

#endif
