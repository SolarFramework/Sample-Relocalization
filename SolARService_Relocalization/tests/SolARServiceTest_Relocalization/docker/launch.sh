#!/bin/sh

# Get Relocalization Service URL from parameters
if [ "$1" ]
then
    echo "Relocalization Service URL = $1"
else
    echo "You need to give Relocalization Service URL as parameter!"
    exit 1
fi

# Set Relocalization Service URL
export RELOCALIZATION_SERVICE_URL=$1

# Get host IP for display
if [ "$2" ]
then
    echo "Display IP = $2"
else
    echo "You need to give host IP address for display as second parameter!"
    exit 1
fi

# Set Display IP
export DISPLAY=$2:0.0

# Set application log level
# Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
export SOLAR_LOG_LEVEL=INFO

docker rm -f solarservicerelocalizationclient
docker run -it -d -e DISPLAY -e RELOCALIZATION_SERVICE_URL -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARServiceRelocalizationClient" -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarservicerelocalizationclient artwin/solar/services/relocalization-client:latest
