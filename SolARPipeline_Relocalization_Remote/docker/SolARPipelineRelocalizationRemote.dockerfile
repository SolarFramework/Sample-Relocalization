FROM ubuntu:18.04
MAINTAINER Christophe Cutullic christophe.cutullic@b-com.com

## Configure Ubuntu environment
RUN apt-get update -y
RUN apt-get install -y libgtk-3-0
RUN apt-get install -y libva-dev
RUN apt-get install -y libvdpau-dev

## Copy SolARPipelineRelocalizationRemote app files
RUN mkdir SolARPipelineRelocalizationRemote

## Data files (fbow vocabulary)
RUN mkdir SolARPipelineRelocalizationRemote/data
RUN mkdir SolARPipelineRelocalizationRemote/data/fbow_voc
ADD data/fbow_voc/* /SolARPipelineRelocalizationRemote/data/fbow_voc/

## Libraries and modules
RUN mkdir SolARPipelineRelocalizationRemote/modules
ADD modules/* /SolARPipelineRelocalizationRemote/modules/

## Project files
ADD SolARPipeline_Relocalization_Remote /SolARPipelineRelocalizationRemote/
RUN chmod +x /SolARPipelineRelocalizationRemote/SolARPipeline_Relocalization_Remote
RUN mkdir .xpcf
ADD *.xml /.xpcf
ADD docker/start_server.sh .
RUN chmod +x start_server.sh

## Set application gRPC server url
ENV XPCF_GRPC_SERVER_URL=0.0.0.0:8080
## Set application gRPC max receive message size
ENV XPCF_GRPC_MAX_RECV_MSG_SIZE=7000000
## Set application gRPC max send message size
ENV XPCF_GRPC_MAX_SEND_MSG_SIZE=2000000

## Set url to Map Update Service
ENV XPCF_GRPC_MAP_UPDATE_URL=map-update-pipeline.artwin.svc.cluster.local:80

## Set application log level
## Log level expected: DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING
ENV SOLAR_LOG_LEVEL=INFO

## Run Server
CMD [ "./start_server.sh"  ]
