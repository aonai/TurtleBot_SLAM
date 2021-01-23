# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++?  
A class includes libraries, whereas a struct only has variables stored in itself. Also, a class can store members as private so that they can only be accessed within itself. 

2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?  
     Because Vector2D is used to stored x and y as related information, and they can vary independently, but Transform2D class needs to store functions and private members that are initialized differently using different constructures. (as stated in C.1, C.2, and C.8)

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?  
     Because these constructors only receive one argument, so they need to be explicit to avoid unexpected results. (refer to C.46)

4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        1. Use a different struct NormalVector2D to store normalized vector.
        2. Use a helper function that outputs a struct NormalVector2D that is calculated from non-normalized Vector2D
        3. Use a class that stores one non-normalized Vector2D, one normalized Vector2D, and functions to convert between the two. 
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        1. This is intuitive, but the code needs to check whether the input is normalized. 
        2. This is easy to implement, but the user will not be able to extract information other than x and y. 
        3. Using class can store private members than may be helpful when implementing functions, but this may get complicated and unnecessary.
   - Which of the methods would you implement and why?
        I would implement method 2, because it's easy to implement and short. 

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?   
   inv() returns a new Transform2D class object that has a constant Vector2D member, but when using operator *=, it is expected that the Transform2D object needs to change its Vector2D member.