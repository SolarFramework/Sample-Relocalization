# ARCloud Relocalization Service 
[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

## Introduction

The purpose of this documentation is to present the **ARCloud Relocalization Service**: a computer vision pipeline dedicated to Augmented Reality (AR) applications and ready to be deployed on a Cloud architecture.

First, we will present the relocalization pipeline on which this remote service is based. Then, we will describe how to configure and deploy the service on a Cloud architecture. And finally, we will explain how to test this relocalization service using a sample application.

## Contents of the Relocalization Service package

The Map Udpate Service is delivered as a package containing all the files you need to deploy and test this computer vision pipeline on your own Cloud architecture.

This package includes:

* the **Docker image** of the remote service, available on a public docker repository (https://artifact.b-com.com/webapp/#/artifacts/browse/tree/General/solar-docker-local/relocalization/0.10.0/relocalization-service/latest)

* the **Kubernetes manifest file** to deploy the service on your own Cloud architecture ( https://artifact.b-com.com/webapp/#/artifacts/browse/tree/General/solar-helm-virtual/relocalization-service-manifest.yaml)

* **a test client applications**, grouped in a compressed file (https://artifact.b-com.com/webapp/#/artifacts/browse/tree/General/solar-generic-local/relocalization-mapping-service-tests/relocalization_mapping_service_test_sample.tar.gz) which contains:

  * the **client application** : _SolARServiceTest_Relocalization_
  * the **client configuration** : _SolARServiceTest_Relocalization_conf.xml_
  * a **script to launch the client application**: _start_client.sh_
  * all the **libraries needed by the test application**, stored in the _modules_ folder
  * a **script to install the data** needed by the two test applications: _install_data.sh_

> :information_source: The complete projects of these two test applications are available on the **SolAR Framework GitHub**:
> * https://github.com/SolarFramework/Sample-Relocalization/tree/develop/SolARService_Relocalization/tests/SolARServiceTest_Relocalization

> :warning: The Relocalization Service Docker image, based on Ubuntu 18.04, is completely independant from other external resources, and can be deployed on any Cloud infrastructure that supports Docker containers.

> :warning: The test client application were built on a **Linux** operating system (Ubuntu 18.04) and must be used on that system only.


## The Relocalization Pipeline 

The goal of the Relocalization Pipeline is to estimate the pose of a camera from an image it has captured. For that, this service must carry out the following steps: 

* **Get the current global map***: This map is managed by the *Map Update Service* (which must therefore already be deployed on your Cloud infrastructure). So, the relocalization pipeline requests the Map Update service to obtain the entire current map and all associated data.
* **Keyframe retrieval**: Each time the relocalization pipeline receives an image for which is has to estimate its pose, it will try to find the similar keyframes stored in the global map data. Several candidate keyframes, which should be captured from viewpoints close to the viewpoint of the input image, can be retrieved if the similarity score is high enough.
* **Keypoint matching**: For each candidate keyframe, the relocalization pipeline will try to match their keypoints with the one detected in the input image. The matches are found thanks to the similarity of the descriptors attached to the keypoints.

To initialize the relocalization pipeline processing, a device must give **the characteristics of the camera** it uses (resolution, focal).

Then, the pipeline is able to process images. To do this, <u>input data</u> is required:

* **the images captured by the device** (images are sent to the pipeline one by one)

And, if the processing is successful, the <u>output data</u> is:

* the **new pose** calculated by the pipeline 
* the **confidence score** of this new pose

To facilitate the use of this pipeline by any client application embedded in a device, it offers a simple interface based on the _SolAR::api::pipeline::IRelocalizationPipeline_ class (see https://solarframework.github.io/create/api/ for interface definition and data structures). 

This interface is defined as follows:

```cpp
    /// @brief Initialization of the pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode init() override;
```

```cpp
    /// @brief Set the camera parameters
    /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
    /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode setCameraParameters(const SolAR::datastructure::CameraParameters & cameraParams) override;
```

```cpp
    /// @brief Start the pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the stard succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode start() override;
```

```cpp
    /// @brief Stop the pipeline.
    /// @return FrameworkReturnCode::_SUCCESS if the stop succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode stop() override;
```

```cpp
    /// @brief Request the relocalization pipeline to process a new image to calculate the corresponding pose
    /// @param[in] image: the image to process
    /// @param[out] pose: the new calculated pose
    /// @param[out] confidence: the confidence score
    /// @return FrameworkReturnCode::_SUCCESS if the processing is successful, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image, SolAR::datastructure::Transform3Df& pose, float_t & confidence) override;
```

The relocalization pipeline project is available on the **SolAR Framework GitHub** (multithreading version): https://github.com/SolarFramework/Sample-Relocalization/tree/develop/SolARService_Relocalization

The map update pipeline project is available on the **SolAR Framework GitHub**: https://github.com/SolarFramework/Sample-MapUpdate/tree/develop/SolARService_MapUpdate


## The Relocalization Service

warning: In order to be able to perform the instructions presented in this part, you must have installed the Kubernetes command-line tool, **_kubectl_**, on your computer: https://kubernetes.io/docs/tasks/tools/

The **Docker image** of this service is available on this public Docker repository: _solar-docker-local.artifact.b-com.com/relocalization/0.10.0/relocalization-service:latest_

A **Kubernetes manifest file sample** is available at this public URL: https://artifact.b-com.com/webapp/#/artifacts/browse/tree/General/solar-helm-virtual/relocalization-service-manifest.yaml

So, you can adapt this manifest sample (in YAML format) to deploy the service:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: relocalization-service
spec:
  replicas: 1
  selector:
    matchLabels:
      app: relocalization-service
  template:
    metadata:
      labels:
        app: relocalization-service
    spec:
      imagePullSecrets:
      - name: regcredsupra
      containers:
      - name: relocalization-service
        image: solar-docker-local.artifact.b-com.com/relocalization/0.10.0/relocalization-service:latest
        env:
        - name: SOLAR_LOG_LEVEL
          value: INFO
        - name: XPCF_GRPC_MAX_RECV_MSG_SIZE
          value: "7000000"
        - name: XPCF_GRPC_MAX_SEND_MSG_SIZE
          value: "2000000"
        - name: XPCF_GRPC_MAP_UPDATE_URL
          value: map-update-service.artwin.svc.cluster.local:80          
---
kind: Service
apiVersion: v1
metadata:
  name: relocalization-service
  labels:
    app: relocalization-service
spec:
  type: NodePort
  selector:
    app: relocalization-service
  ports:
  - name: http
    port: 80
    targetPort: 8080
    nodePort: 31889
---
apiVersion: v1
kind: ServiceAccount
metadata:
  name: relocalization-service
---
apiVersion: rbac.authorization.k8s.io/v1
kind: ClusterRoleBinding
metadata:
  name: relocalization-service-rolebinding
roleRef:
  apiGroup: rbac.authorization.k8s.io
  kind: ClusterRole
  name: cluster-admin
subjects:
- kind: ServiceAccount
  name: relocalization-service
  namespace: relocalization-service.artwin.b-com.com:q
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  annotations:
    ingress.kubernetes.io/force-ssl-redirect: "true"
    kubernetes.io/ingress.class: nginx
    nginx.ingress.kubernetes.io/backend-protocol: "GRPC"
  name:
    relocalization-service
spec:
  tls:
  - hosts:
      - relocalization-service.artwin.b-com.com
  rules:
  - host: relocalization-service.artwin.b-com.com
    http:
      paths:
      - backend:
          service:
            name: relocalization-service
            port:
              number: 80
        path: /
        pathType: Prefix
```

<u>**Deployment part** (_kind: Deployment_)</u>

In this part of the manifest document, you need to define the service deployment parameters:

* name of the deployment
```yaml
metadata:
  name: relocalization-service
```

* number of replica pods for this deployment
```yaml
  replicas: 1
```

* label linked to the deployment object, used for identification
```yaml
  selector:
    matchLabels:
      app: relocalization-service
  template:
    metadata:
      labels:
        app: relocalization-service
 ```

* **container parameters**: this part is important because it defines the Docker image you want to deploy (_image: solar-docker-local.artifact.b-com.com/relocalization/0.10.0/relocalization-service:latest_), 
and some parameters of the Docker container used at runtime:

  * SOLAR_LOG_LEVEL: the log level of the relocalization service (DEBUG, CRITICAL, ERROR, INFO, TRACE, WARNING)
  * XPCF_GRPC_MAX_RECV_MSG_SIZE: the maximum size, in bytes, of gRPC received messages ("-1" for maximum value)
  * XPCF_GRPC_MAX_SEND_MSG_SIZE: the maximum size, in bytes, of gRPC sent messages ("-1" for maximum value)
  * XPCF_GRPC_MAP_UPDATE_URL: <u>the local cluster URL to the Map Update Service</u> (**this URL depends on your cluster configuration**)

```yaml
      containers:
      - name: relocalization-service
        image: solar-docker-local.artifact.b-com.com/relocalization/0.10.0/relocalization-service:latest
        env:
        - name: SOLAR_LOG_LEVEL
          value: INFO
        - name: XPCF_GRPC_MAX_RECV_MSG_SIZE
          value: "7000000"
        - name: XPCF_GRPC_MAX_SEND_MSG_SIZE
          value: "2000000"
        - name: XPCF_GRPC_MAP_UPDATE_URL
          value: map-update-service.artwin.svc.cluster.local:80   
```


<u>**Service part** (_kind: Service_)</u>

This part defines the name of the service, and, more important, **the service node port**, which will be used by client applications to access its interface (in this example _31887_). The service port and target port must not be changed because they correspond to the definition of the ports of the Docker container (respectively _80_ and _8080_).

```yaml
kind: Service
apiVersion: v1
metadata:
  name: relocalization-service
  labels:
    app: relocalization-service
spec:
  type: NodePort
  selector:
    app: relocalization-service
  ports:
  - name: http
    port: 80
    targetPort: 8080
    nodePort: 31889
```

<u>**Ingress part** (_kind: Ingress_)</u>

This last part is dedicated to the Ingress object that manages external access to the service. It defines important parameters such as **backend protocol**(_GRPC_ for the relocalization service), **host name** (_relocalization-service.artwin.b-com.com_ in this example) and **service port number** as defined in the _service_ part.

```yaml
kind: Ingress
metadata:
  annotations:
    ingress.kubernetes.io/force-ssl-redirect: "true"
    kubernetes.io/ingress.class: nginx
    nginx.ingress.kubernetes.io/backend-protocol: "GRPC"
  name:
    relocalization-service
spec:
  tls:
  - hosts:
      - relocalization-service.artwin.b-com.com
  rules:
  - host: relocalization-service.artwin.b-com.com
    http:
      paths:
      - backend:
          service:
            name: relocalization-service
            port:
              number: 80
        path: /
        pathType: Prefix
```

TIP: You can test this service locally on your computer by following these steps:

1. Download the Service image to your own Docker engine: 

```command
docker pull solar-docker-local.artifact.b-com.com/relocalization/0.10.0/relocalization-service:latest
```

2. Get the launch script corresponding to your Operating System (Windows or Linux) from SolAR GitHub: 

**Linux**: https://github.com/SolarFramework/Sample-Relocalization/blob/develop/SolARService_Relocalization/docker/launch.sh

**Windows**: https://github.com/SolarFramework/Sample-Relocalization/blob/develop/SolARService_Relocalization/docker/launch.bat

3. Execute the launch script (by default, the log level is set to "INFO" and the service port is set to **"50052"**: you can modify this values inside the script file)
=> **You must give the Map Update Service URL as a script parameter**

4. Check the logs of the service: 

```command
docker logs solarservicerelocalization
```

You should see something like this: 

```command
MAPUPDATE_SERVICE_URL defined: 172.17.0.1:50053
Try to replace the MapUpdate Service URL in the XML configuration file...
XML configuration file ready
[2021-09-22 08:58:57:010581] [info] [    8] [main():137] Load modules configuration file: /.xpcf/SolARService_Relocalization_modules.xml
[2021-09-22 08:58:57:020090] [info] [    8] [main():146] Load properties configuration file: /.xpcf/SolARService_Relocalization_properties.xml
[2021-09-22 08:58:57:712268] [info] [    8] [main():161] LOG LEVEL: INFO
[2021-09-22 08:58:57:712347] [info] [    8] [main():163] GRPC SERVER ADDRESS: 0.0.0.0:8080
[2021-09-22 08:58:57:712353] [info] [    8] [main():165] GRPC SERVER CREDENTIALS: 0
[2021-09-22 08:58:57:712356] [info] [    8] [main():171] GRPC MAX RECEIVED MESSAGE SIZE: 7000000
[2021-09-22 08:58:57:712378] [info] [    8] [main():178] GRPC MAX SENT MESSAGE SIZE: 2000000
[2021-09-22 08:58:57:712381] [info] [    8] [main():182] XPCF gRPC server listens on: 0.0.0.0:8080
```

If you want to test this service locally with one of our test applications (presented later in this document), you can use the **Docker bridge IP address** (and the port define in the launch script) given by this command ("Gateway" value): 

```command
docker network inspect bridge
[
    {
        "Name": "bridge",
        "Id": "7ec85993b0ab533552db22a0a61794f0051df5782703838948cdac82d4069674",
        "Created": "2021-09-22T06:51:14.2139067Z",
        "Scope": "local",
        "Driver": "bridge",
        "EnableIPv6": false,
        "IPAM": {
            "Driver": "default",
            "Options": null,
            "Config": [
                {
                    "Subnet": "172.17.0.0/16",
                    "Gateway": "172.17.0.1"
                }
            ]
        },
...
```

### kubectl commands

To deploy or re-deploy the map udpate service in your infrastructure, when your manifest file is correctly filled in, you can use the **kubectl** command-line tool to:

* define your **Kubernetes context** in a **.conf** file (cluster name, namespace, server address and port, user, certificates, etc.) like this:

```yaml
    apiVersion: v1
    clusters:
    - cluster:
        certificate-authority-data: ...
        server: https://...
    name: kubernetes
    contexts:
    - context:
        cluster: kubernetes
        namespace: artwin
        user: kubernetes-admin
    name: cluster-mapudpate
    current-context: cluster-relocalization
    kind: Config
    preferences: {}
    users:
    - name: kubernetes-admin
    user:
        client-certificate-data: ...
        client-key-data: ...
```

and set it as default configuration:
```command
export KUBECONFIG=path/to/your_configuration.conf
```

- **set the context to use**:
```command
kubectl config use-context [context name]
```

you can also specify the default namespace:
```command
kubectl config set-context --current --namespace=[namespace]
```

- **deploy your service**:
```command
kubectl apply -f [path/to/your_manifest.yaml]
```

- **check your deployments**:
```command
kubectl get deployments
NAME               READY   UP-TO-DATE   AVAILABLE   AGE
relocalization-service   1/1     1            1           21d
```

- **check your services**:
```command
kubectl get services
NAME                      TYPE       CLUSTER-IP       EXTERNAL-IP   PORT(S)          AGE
relocalization-service          NodePort   10.107.117.85    <none>        80:31887/TCP     30d
```

- **check your pods**:
```command
kubectl get pods
NAME                                READY   STATUS    RESTARTS   AGE
relocalization-service-84bc74d954-6vc7v   1/1     Running   1          7d15h
```

- **visualize the logs of a pod**:
```command
kubectl logs -f [pod name]
```

- **restart a pod**: to do that, you can for example change an environment variable of the pod, to force it to restart
```command
kubectl set env deployment [deployment name] SOLAR_LOG_LEVEL=INFO
```

information: You can find more kubectl commands on this web site: https://kubernetes.io/docs/reference/kubectl/cheatsheet/

## Relocalization Service test samples

This application is used to test the pose calculation carried out remotely by the Relocalization Service. It uses a full set of pre-captured images taken from a Hololens device, included in this sample package.

To test the service, this application reads each image from the Hololens data files and, every "x" images, requests the Relocalization Service to process the current one, using this method from its interface:

```cpp
    /// @brief Request the relocalization pipeline to process a new image to calculate the corresponding pose
    /// @param[in] image: the image to process
    /// @param[out] pose: the new calculated pose
    /// @param[out] confidence: the confidence score
    /// @return FrameworkReturnCode::_SUCCESS if the processing is successful, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image, SolAR::datastructure::Transform3Df& pose, float_t & confidence) override;
```

Traces are displayed at runtime to follow the progress of the application. In addition, it displays each image read in a specific window to be able to visually follow the path of the Hololens device. 

### Launch the test application

First, you have to install the test application image on your Docker engine: 

```command
docker pull solar-docker-local.artifact.b-com.com/tests/0.10.0/relocalization-client:latest
```

Then, get the script file used to launch the Docker container of the test application from one of these URL: 

**Linux**: https://github.com/SolarFramework/Sample-Relocalization/blob/develop/SolARService_Relocalization/tests/SolARServiceTest_Relocalization/docker/launch.sh

**Linux VM**: https://github.com/SolarFramework/Sample-Relocalization/blob/develop/SolARService_Relocalization/tests/SolARServiceTest_Relocalization/docker/launch_vm.sh

**Windows**: https://github.com/SolarFramework/Sample-Relocalization/blob/develop/SolARService_Relocalization/tests/SolARServiceTest_Relocalization/docker/launch.bat

You can now launch the Docker container using the script file, giving **your Relocalization Service URL** and, <u>if you do not use a Linux Virtual Machine</u>, **your computer local IP address (to export graphical display)** as parameter: 

```command
launch.sh [relocalization_service_url] [host_IP_address]
launch_vm.sh [relocalization_service_url]
launch.bat [relocalization_service_url] [host_IP_address]

For example: 
```command
launch.bat 172.17.0.1:50052 192.168.56.1
```

CAUTION: On Windows host, do not forget to start the X Server manager before running the test application

Then, you can verify that the application is running correctly by looking at its traces, with this Docker command:

```command
docker logs [-f] solarservicerelocalizationclient
```

And you will see something like this: 

```command
RELOCALIZATION_SERVICE_URL defined: 172.17.0.1:50052
Try to replace the Relocalization Service URL in the XML configuration file...
XML configuration file ready
[2021-10-20 14:52:34:711132] [info] [    8] [main():101] Get component manager instance
[2021-10-20 14:52:34:711374] [info] [    8] [main():105] Load Client Remote Relocalization Service configuration file: /.xpcf/SolARServiceTest_Relocalization_conf.xml
[2021-10-20 14:52:34:711910] [info] [    8] [main():109] Resolve IRelocalizationPipeline interface
[2021-10-20 14:52:35:270449] [info] [    8] [main():114] Resolve components used
[2021-10-20 14:52:35:302757] [info] [    8] [main():121] Set relocalization service camera parameters
[2021-10-20 14:52:35:307598] [info] [    8] [main():129] Start relocalization pipeline process
[2021-10-20 14:52:35:310974] [info] [    8] [main():136]
***** Control+C to stop *****
[2021-10-20 14:52:35:716387] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:52:43:432279] [info] [    8] [main():169] Failed to calculate pose for this image
[2021-10-20 14:52:57:486997] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:52:58:317535] [info] [    8] [main():169] Failed to calculate pose for this image
[2021-10-20 14:53:11:988718] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:53:12:932932] [info] [    8] [main():169] Failed to calculate pose for this image
[2021-10-20 14:53:26:661736] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:53:27:699770] [info] [    8] [main():169] Failed to calculate pose for this image
[2021-10-20 14:53:41:562936] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:53:42:334819] [info] [    8] [main():169] Failed to calculate pose for this image
[2021-10-20 14:53:48:776327] [info] [    8] [main():162] Send an image to relocalization pipeline
[2021-10-20 14:53:49:454503] [info] [    8] [main():165] New pose calculated by relocalization pipeline
...
```

And you will see the images loaded from the Hololens dataset in a dedicated window.

## The Relocalization, Mapping (and Map Update) Services test application

This application can be used to test both Relocalization and Mapping services. In addition, as these two services rely on the Map Update service, this third one can also be tested using this example. This program uses a full set of pre-captured images taken from a Hololens device, included in the sample package.

First of all, this test program tries to determine the transformation matrix between the Hololens coordinate system and that of the global map. To do that, it uses the 'n' first images from the Hololens dataset to request the Relocalization service for pose calculation, based on the global map provided by the Map Update service. If this treatment is successful, the program is able to compare the original Hololens pose with the new pose from the Relocalization service, and to deduce the transformation.

For this step, this method of the Relocalization Service interface is called:

```cpp
    /// @brief Request the relocalization pipeline to process a new image to calculate the corresponding pose
    /// @param[in] image: the image to process
    /// @param[out] pose: the new calculated pose
    /// @param[out] confidence: the confidence score
    /// @return FrameworkReturnCode::_SUCCESS if the processing is successful, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image, SolAR::datastructure::Transform3Df& pose, float_t & confidence) override;
```

Then, the test application sends each pair of image and pose from the Hololens dataset to the Mapping service, applying the transformation matrix if known. Thus, the Mapping pipeline will be able to constitute a new local sparse map completed by the precise poses corresponding to the movements of the Hololens device.

For this step, this method of the Mapping Service interface is called:

```cpp
    /// @brief Request to the mapping pipeline to process a new image/pose
    /// Retrieve the new image (and pose) to process, in the current pipeline context
    /// (camera configuration, point cloud, key frames, key points)
    /// @param[in] image: the input image to process
    /// @param[in] pose: the input pose to process
    /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode mappingProcessRequest(const SRef<datastructure::Image> image,
                                              const datastructure::Transform3Df & pose) override;
```

At the end of the program, when all the Hololens images have been analyzed, the Mapping service sends the new local map to the Map Update service, to consolidate and enrich the global sparse map.

Traces are displayed at runtime to follow the progress of the application. In addition, it displays each image read in a specific window to be able to visually follow the path of the Hololens device.

TIP: At any time during the execution of this example, you can use the **Mapping Service viewer client** to visualize the current local map being creating.

TIP: At the end of this application, you can use the **Map Update Service test application** to view the new global map.

### Launch the test application

First, you have to install the test application image on your Docker engine: 

```command
docker pull solar-docker-local.artifact.b-com.com/tests/0.10.0/mapping-multi-relocalization-client:latest
```

Then, get the script file used to launch the Docker container of the test application from one of these URL: 

**Linux**: https://github.com/SolarFramework/Sample-Mapping/blob/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Relocalization/docker/launch.sh

**Linux VM**: https://github.com/SolarFramework/Sample-Mapping/blob/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Relocalization/docker/launch_vm.sh

**Windows**: https://github.com/SolarFramework/Sample-Mapping/blob/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Relocalization/docker/launch.bat

You can now launch the Docker container using the script file, giving **your Relocalization Service URL**, **your Mapping Service URL** and, <u>if you do not use a Linux Virtual Machine</u>, **your computer local IP address (to export graphical display)** as parameter: 

```command
launch.sh [relocalization_service_url] [mapping_service_url] [host_IP_address]
launch_vm.sh [relocalization_service_url] [mapping_service_url]
launch.bat [relocalization_service_url] [mapping_service_url] [host_IP_address]
```

For example: 
```command
launch.bat 172.17.0.1:50052 172.17.0.1:50051 192.168.56.1
```

You can also change the default Hololens image data set used for the test by specifying the "B" data set ("A" used by default)

For example: 
```command
launch.bat 172.17.0.1:50052 172.17.0.1:50051 192.168.56.1 B
```

CAUTION: On Windows host, do not forget to start the X Server manager before running the test application

Then, you can verify that the application is running correctly by looking at its traces, with this Docker command:

```command
docker logs [-f] solarservicemappingmultirelocalizationclient
```

And you will see something like this: 

```command
RELOCALIZATION_SERVICE_URL defined: 172.17.0.1:50052
Replace the Relocalization Service URL in the XML configuration file...
MAPPING_SERVICE_URL defined: 172.17.0.1:50051
Replace the Mapping Service URL in the XML configuration file...
HOLOLENS_DATA_SET defined: A
Path to image set to: ./data/data_hololens/loop_desktop_A
Replace the path to image data in the XML configuration file...
XML configuration file ready
[2021-08-02 16:42:29:986811] [info] [24091] [main():228] Get component manager instance
[2021-08-02 16:42:29:987168] [info] [24091] [main():232] Load configuration file: SolARServiceTest_Mapping_Multi_Relocalization_conf.xml
[2021-08-02 16:42:29:987681] [info] [24091] [main():236] Resolve IMappingPipeline interface
[2021-08-02 16:42:30:018976] [info] [24091] [main():239] Resolve IRelocalizationPipeline interface
[2021-08-02 16:42:30:041947] [info] [24091] [main():250] Remote producer client: AR device component created
[2021-08-02 16:42:30:042243] [info] [24091] [main():253] Remote producer client: AR device component created
[2021-08-02 16:42:30:055965] [info] [24091] [main():264] Remote mapping client: Init mapping pipeline result = SUCCESS
[2021-08-02 16:42:30:056461] [info] [24091] [main():273] Remote mapping client: Set mapping pipeline camera parameters result = SUCCESS
[2021-08-02 16:42:30:487307] [info] [24091] [main():277] Remote relocalization client: Init relocalization pipeline result = SUCCESS
[2021-08-02 16:42:30:489220] [info] [24091] [main():286] Remote relocalization client: Set relocalization pipeline camera parameters result = SUCCESS
[2021-08-02 16:42:30:489261] [info] [24091] [main():288] Remote mapping client: Start remote mapping pipeline
[2021-08-02 16:42:30:490200] [info] [24091] [main():292] Start remote mapping client thread
[2021-08-02 16:42:30:490391] [info] [24091] [main():297] Start remote relocalization client thread
[2021-08-02 16:42:30:490508] [info] [24091] [main():302] Read images and poses from hololens files
[2021-08-02 16:42:30:490538] [info] [24091] [main():304]
***** Control+C to stop *****
[2021-08-02 16:42:31:082000] [info] [24091] [main():330] Add image to input drop buffer for relocalization
[2021-08-02 16:42:31:082029] [info] [24091] [main():341] Add pair (image, pose) to input drop buffer for mapping
[2021-08-02 16:42:31:082150] [info] [24102] [operator()():82] Mapping client: Send (image, pose) to mapping pipeline
[2021-08-02 16:42:31:082599] [info] [24103] [operator()():107] Relocalization client: Send image to relocalization pipeline
[2021-08-02 16:42:31:389366] [info] [24091] [main():341] Add pair (image, pose) to input drop buffer for mapping
[2021-08-02 16:42:31:389448] [info] [24102] [operator()():82] Mapping client: Send (image, pose) to mapping pipeline
...
```

If the **relocalization processing is successful**, you will see the following trace: 

```command
[2021-08-02 16:42:32:544970] [info] [24103] [operator()():111] => Relocalization succeeded
[2021-08-02 16:42:32:544999] [info] [24103] [operator()():113] Hololens pose:    0.996649  -0.0777834   0.0252523  -0.0062405
  -0.045278   -0.781977   -0.621658 -0.00958775
  0.0681019    0.618432   -0.782881   -0.143014
          0           0           0           1
[2021-08-02 16:42:32:545107] [info] [24103] [operator()():114] World pose:    0.996321  -0.0810918    0.027727 -0.00972971
 -0.0469172   -0.786846   -0.615364  -0.0186298
  0.0717178    0.611799   -0.787756   -0.138227
          0           0           0           1
[2021-08-02 16:42:32:545174] [info] [24103] [operator()():118] Transformation matrix from Hololens to World:    0.999993   0.0010635 -0.00400569 -0.00405193
  -0.001096    0.999969 -0.00805077  -0.0102006
 0.00399664  0.00805406     0.99996  0.00488381
          0           0           0           1
```

At any time, you will see the images loaded from the Hololens dataset in a dedicated window.

NOTE: The complete projects of these test applications are available on the **SolAR Framework GitHub**: 
* https://github.com/SolarFramework/Sample-MapUpdate/tree/develop/SolARService_MapUpdate/tests/SolARServiceTest_MapUpdate 
* https://github.com/SolarFramework/Sample-Mapping/tree/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Producer 
* https://github.com/SolarFramework/Sample-Mapping/tree/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Viewer 
* https://github.com/SolarFramework/Sample-Mapping/tree/develop/Mapping/SolARService_Mapping_Multi/tests/SolARServiceTest_Mapping_Multi_Relocalization
