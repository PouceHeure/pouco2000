# Pouco2000

:warning: in development 

- [Pouco2000](#pouco2000)
  - [ROS Package](#ros-package)
    - [Setup package](#setup-package)
      - [Place it](#place-it)
      - [Compile it](#compile-it)
    - [Use package](#use-package)
      - [Controller msg](#controller-msg)
    - [Documention](#documention)
  - [Arduino Library](#arduino-library)
    - [Setup library](#setup-library)
      - [Place it](#place-it-1)
      - [Place ros_lib](#place-roslib)
    - [Use library](#use-library)
    - [Warning](#warning)
  - [Modelization](#modelization)
  - [Examples](#examples)
 
## ROS Package 
A project ros has been developped, called *pouco2000_ros*, this package receives data from electronic part, and regroups these message into one msg. 

The package is generally based on 2 librairies:
- *pouco2000*, principal library, grouping Controller class definition. 
- *pouco2000_debug*, allowing to develope easily the ros part. I decided to let this part in the release version, allowing to user to develope efficacely depend packages.

3 others libaries has been developed. 
- pouco2000_introspection, filter data from controller msg and publish data 
- pouco2000_extractor, extract data from controller msg 
- pouco2000_monitor, grouping methods and classes allowing to create a monitor.

### Setup package

#### Place it 

The *pouco2000_ros* need to be placed inside your ros working space. It's possible to create a symbolic link inside your ros ws. 

``` shell
USER$ ln -s {package/path} {ros_ws/path}
```

(It's also possible to copy and paste the package)

#### Compile it

Once moved or linked, the package need to be compiled. It's possbile to use catkin_make or catkin build. 

```shell
** go inside in your working space (at the root level) **
USER$ catkin_make pouco2000_ros
```
or 
```shell
** go inside in your working space (at any level) **
USER$ catkin build pouco2000_ros
```

### Use package 

#### Controller msg 

The project is essentially based on a principal msg: pouco2000::Controller

```
Header header
Buttons buttons
SwitchsOnOff switchs_on_off
SwitchsMode switchs_mode
Potentiometers potentiometers_circle
Potentiometers potentiometers_slider
```

Each part expect header is an array of data. 

### Documention 

a *rosdoc* command has been added to the CMakeLists, so when you will compile the package, a documentation folder will be added to package including a doxygen documentation.

## Arduino Library 

An arduino library has been developed, allowing to create easily a code.

### Setup library 

#### Place it 

Like ROS package, the library need to be placed at the good place. It's possible to create a symbolic link.  

```shell 
USER$ ln -s {arduino_lib/path} {arduino/path/libraries/}
```

#### Place ros_lib

The project uses ROSSERIAL package. Once the *pouco2000_ros* has been compiled, the header's msgs need to placed into arduino libraries. 

The lib_ros generation can be done, by this command: 

```shell
USER$ rosrun rosserial_arduino make_libraries.py {arduino/path/libraires}
```

For more information about this package: [http://wiki.ros.org/rosserial](http://wiki.ros.org/rosserial) 

### Use library  

For each field (buttons, switchs on off...), a handle object need to be created.  

```C++
/**
 * @brief Construct a new Handle object
 * 
 * @param topic topic where the message will be published
 * @param connections array of connections
 * @param n_connections number of connections 
 * @param is_digital if the field use digital or analog port 
 */
Handle<T_field,T_data,T_msg>::Handle(const char* topic,int* connections,int n_connections,bool is_digital)
```
In the setup method, the handle need to call a setup method. 

```C++
/**
 * @brief setup the current handle, declare the publisher to the NodeHandle and 
 * set the pinMode of each pin to INPUT  
 * @param nh current nodehandle 
 */
void setup(ros::NodeHandle& nh);
```

In the loop method, the handle need to call a update method.

```C++ 
/**
* @brief update msg used by the handle in checking state of pin 
* 
*/
void update();
```

In the libraray, typical typedefs are already defined. 

```C++

typedef Handle<Switch,pouco2000_ros::SwitchsOnOff::_data_type,pouco2000_ros::SwitchsOnOff> HandleSwitchsOnOff;

typedef Handle<SwitchMode,pouco2000_ros::SwitchsMode::_data_type,pouco2000_ros::SwitchsMode> HandleSwitchsMode;

typedef Handle<Button,pouco2000_ros::Buttons::_data_type,pouco2000_ros::Buttons> HandleButtons;

typedef Handle<Potentiometer,pouco2000_ros::Potentiometers::_data_type,pouco2000_ros::Potentiometers> HandlePotentiometers;
```

So, for each field you need:
1. varibales definition (outside of setup and loop method) 
   1. create a pin array, defining pin used by this field
   2. create a handle for this field
2. inside setup method 
   1. call the setup of the handle with the current nodehandle
3. inside loop method: 
   1. call the update method of the handle 

> Some examples has beed developed and added to the librarie. Theses examples can be loaded from the arduino IDE (file -> Examples -> pouco2000_ard). 

### Warning 
The library has been developed and tested on the following boards: 
- arduino UNO
- arduino nano 

## Modelization 

## Examples  
