set(LIB_NAME swarmus-hiveconnect-network-ros)
set(LIB_ALIAS SwarmUS::HiveConnect::Network::Platform)


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})