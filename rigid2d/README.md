# Rigid 2D Transformation Library
A library for handling transformations in SE(2)

# Conceptual Questions
1. The difference between a class and a struct in C++ is in their members' default visibility. Classes are private, which means you can't access it's members outside the scope of its class. Structs are public, which means you can access it outside of the scope of the struct.

2. Vector2D is a struct because we want to be able to access its members from outside the scope of Vector2D. For example, when we write methods for Transform2D. According to C++ Core Guideline C.2: "Use class if the class has an invariant, use struct if the data members can vary independently." We know that the data members of Vector2D can vary independently of each other. However the members of Transform2D cannot vary independently of each other, they are dependent on the original dimensions since they represent a rigid body transformation.

According to C++ Core Guideline C.3: "Represent the distinction between an interface and an implementation using a class." Using a rigid body transformation changes the representation of the coordinates.

According to C++ Core Guideline C.9: "Minimize exposure of members." We want to enforce the relation among members that result in a rigid body transformation, therefore we make them private. This means that we would make Transform2D a class. There aren't any strict relations that we want to necessarily enforce when inputting a 2-dimensional vector. Therefore, we can make it a struct.

3. According to C++ Core Guideline C.46: "By default, declare single-argument constructors explicit." The constructors in Transform2D for pure translation and pure rotation only require a single-argument. Therefore, they are explicitly declared. This avoids unintended conversions.

4. In order to find the unit vector in the direction of a given Vector2D, we must first take the magnitude of the vector. We then divide the x and y components by the magnitude. There are several ways we can implement this. The first method is very straightforward, and that is by manually taking the magnitude and dividing the x and y components. This is definitely not the most effective implementation though, since it is a functionality we will be repeating many times in the future.

Another method is by creating a function called normalize(vector) that inputs a Vector2D and outputs a Vector2D. This avoids the repetitiveness of the previous implementation and simplifies the amount of code that we need to write everytime we want to normalize a vector.

The last method, which is the one that I implemented in my rigid2d.cpp, is to create a method in the Vector2D struct called normalize(). I found this to be the best approach (for me) because it is very straightforward as to which Vector2D you are normalizing, since you don't have to worry about an input. You just add ".normalize()" to the end of the Vector2D vector you want to normalize. This also ensures that the normalize functionality will be applied to strictly Vector2D objects, and not other objects like Transform2D's or Twist2D's.

5. According to C++ Core Guidelines, by default we make member functions const. This is why Transform2D::inv() is const. The reason why Transform2D::operator*=() is not const because we are changing the values of the Transform2D that it is being applied to. Therefore, we would not make it constant. 