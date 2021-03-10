SET(INTERFACEINCLUDES
        GenericIndividual.hpp
	GenericIndividual.tcc
	Go3.hpp
)

message( geneva-inteface includes :${INTERFACEINCLUDES})
message( geneva-inteface includes :)

SET ( SAMPLEINDIVIDUALCONFIGS
	# No config files at the moment
)

# This is a workaround for a CLion-problem -- see CPP270 in the JetBrains issue tracker
ADD_CUSTOM_TARGET(CLION_ALL_INTERFACE_HEADERS SOURCES ${INTERFACEINCLUDES})

SET_TARGET_PROPERTIES(
	CLION_ALL_INTERFACE_HEADERS
    PROPERTIES
	CXX_STANDARD 17
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)


# Copy the config files to the build tree for easier testing
FILE ( COPY ${SAMPLEINDIVIDUALCONFIGS} DESTINATION . )

INSTALL ( FILES ${INTERFACEINCLUDES} DESTINATION ${INSTALL_PREFIX_INCLUDES}/geneva-interface)

INSTALL ( FILES ${SAMPLEINDIVIDUALCONFIGS} DESTINATION ${INSTALL_PREFIX_INCLUDES}/geneva-interface)
