ECHO off

REM Get Relocalization Service URL from parameters
IF "%1"=="" (
    ECHO You need to give Relocalization Service URL as first parameter!
    GOTO end
) ELSE (
    ECHO Relocalization Service URL = %1
)

REM Set Relocalization Service URL
SET RELOCALIZATION_SERVICE_URL=%1

REM Get host IP for display
IF "%2"=="" (
    ECHO You need to give host IP address for display as second parameter!
    GOTO end
) ELSE (
    ECHO Set display IP
    SET DISPLAY=%2:0.0
)

ECHO Display IP = %DISPLAY%

REM Set application log level
REM Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
SET SOLAR_LOG_LEVEL=INFO

docker rm -f solarservicerelocalizationclient
docker run -it -d -e DISPLAY -e MAPUPDATE_SERVICE_URL -e SOLAR_LOG_LEVEL -e "SERVICE_NAME=SolARServiceRelocalizationClient" -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --log-opt max-size=50m -e "SERVICE_TAGS=SolAR" --name solarservicerelocalizationclient artwin/solar/services/relocalization-client:latest

:end
