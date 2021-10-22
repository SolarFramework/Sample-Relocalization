#!/bin/sh

## Detect RELOCALIZATION_SERVICE_URL var and use its value
## to set the Relocalization service URL in XML configuration file

cd /SolARServiceMapUpdateClient

if [ -z "$RELOCALIZATION_SERVICE_URL" ]
then
    echo "Error: You must define RELOCALIZATION_SERVICE_URL env var with the MapUpdate Service URL"
    exit 1 
else
    echo "RELOCALIZATION_SERVICE_URL defined: $RELOCALIZATION_SERVICE_URL"
fi

echo "Try to replace the Relocalization Service URL in the XML configuration file..."

sed -i -e "s/RELOCALIZATION_SERVICE_URL/$RELOCALIZATION_SERVICE_URL/g" /.xpcf/SolARServiceTest_Relocalization_conf.xml

echo "XML configuration file ready"

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

export LD_LIBRARY_PATH=.:./modules/

## Start client
./SolARServiceTest_Relocalization_ -f /.xpcf/SolARServiceTest_Relocalization_conf.xml

