# Pouco2000

:warning: in development  
 

## ROS Package 
A project ros has been developped, called *pouco2000_ros*, this package receives data from electronic part, and regroups these message into one msg. 

The package is generally based on 2 librairies *pouco2000* and *pouco2000_debug*. Debug librairie has been developed, allowing to develope easily the ros part. I decided to let this part in the release version, allowing to user to develope efficacely depend packages. 

### Setup package

#### Place it 

The *pouco2000_ros* need to be placed inside your ros working space. It's possible to create a symbolic link inside your ros ws. 

```
ln -s {package/path} {ros_ws/path}
```

(It's also possible to copy and paste the package)

#### Compile it

Once moved or linked, the package need to be compiled. It's possbile to use catkin_make or catkin build. 

```
** go inside in your working space (at the root level) **
catkin_make pouco2000_ros
```
or 
```
** go inside in your working space (at any level) **
catkin build pouco2000_ros
```

### Use package 

(TODO)

### Documention 

a *rosdoc* has been added to the CMakeLists, so when you will compile the package, a documentation folder will be added to package.


## Electronic 



## Modelization 

## Examples  
