# Rigid 2D Transformation Library
A library for handling transformations in SE(2)

# Conceptual Questions
1. The difference between a class and a struct in C++ is in their members' default visibility. Classes are private while structs are public.

2. According to C++ Core Guideline C.2: "Use class if the class has an invariant, use struct if the data members can vary independently." We know that the data members of Vector2D can vary independently of each other. However the members of Transform2D cannot vary independently of each other, they are dependent on the original dimensions since they represent a rigid body transformation.

According to C++ Core Guideline C.3: "Represent the distinction between an interface and an implementation using a class." Using a rigid body transformation changes the representation of the coordinates.

According to C++ Core Guideline C.9: "Minimize exposure of members." We want to enforce the relation among members that result in a rigid body transformation, therefore we make them private. This means that we would make Transform2D a class. There aren't any strict relations that we want to necessarily enforce when inputting a 2-dimensional vector. Therefore, we can make it a struct.

3. According to C++ Core Guideline C.46: "By default, declare single-argument constructors explicit." The constructors in Transform2D for pure translation and pure rotation only require a single-argument. Therefore, they are explicitly declared. This avoids unintended conversions.

4. 

5. 