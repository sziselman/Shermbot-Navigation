# Rigid2D
This package contains two libraries: 1) `rigid2D` and 2) `diff_drive`.

## rigid2D library
### Overview
This library handles transformations in SE(2). This library contains the following:
* `Vector2D` - a struct that contains an `x` and `y` component. Functionalities include normalizing, getting the magnitude and angle, and basic operators.
* `Transform2D` - a class that contains a rigid body transformation in 2 dimensions. Functionalities include applying the transformation to a `Vector2D`, inverting the transform, converting a `Twist2D` to a different frame using the adjoint, and basic operators.
* `Twist2D` - a struct that contains x-velocity `dx`, y-velocity `dy`, and angular velocity `dth` components.

### Main Implementation
Running the following executable,
```
./rigid2d
```
will result in a program that promps the user to enter a series of transforms, vectors, twists, and frames. The result will be several calculations that represent the functionalities of the structs and classes that are contained in the library.

### Testing
Running the following executable,
```
./tests
```
implement unit tests using the `catchw` framework. It will display the number of tests ran and tests passed.

## diff_drive library
