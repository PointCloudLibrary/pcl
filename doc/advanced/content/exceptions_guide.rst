.. _exceptions_guide:

Exceptions in PCL
-----------------
We had a discussion on exceptions writing and handling in PCL. The
grand lines are highlighted here.

Exceptions writing
==================
Any new exception should inherit from PCLException class in
``pcl/exceptions.h``

.. code-block:: cpp

  /** \class MyException
    * \brief An exception that is thrown when I want it.
		*/
		
  class PCL_EXPORTS MyException : public PCLException
  {
    public:
      MyException (const std::string& error_description,
                   const std::string& file_name = "",
                   const std::string& function_name = "",
                   unsigned line_number = 0) throw ()
        : pcl::PCLException (error_description, file_name, function_name, line_number) { }  
  };

Exceptions using
================
For ease of use we provide this macro

.. code-block:: cpp

  #define PCL_THROW_EXCEPTION(ExceptionName, message)
  {
    std::ostringstream s;
    s << message;
    throw ExceptionName(s.str(), __FILE__, "", __LINE__);
  }

Then in your code you can put

.. code-block:: cpp

  if(my_requirements != the_parameters_used_)
	  PCL_THROW_EXCEPTION(MyException, "my requirements are not met " << the_parameters_used);

This will set the file name and the line number thanks to the macro
definition.
Take care of the message it is the most important part of the
exception. You can make profit of the std::ostringstream used in the
macro so you can append variable names to variable values and so on to
make it really explicit.
Also something really important is when the method you are writing
``can`` throw an exception then please in the function documentation
add it this way:

.. code-block:: cpp

  /** Function that does cool stuff
    * \param nb number of points
		* \throws MyException
    */
  void myFunction(int nb);

This will make it available in the generated API documentation so the
person that would use your function knows that she has to deal with
MyException.

Exceptions handling
===================
To properly handle exceptions you need the ``try``... ``catch`` block.

.. code-block:: cpp

  // Here we call myFunction which can throw MyException
  try
  {
	  myObject.myFunction(some_number);
	  // You can put more exceptions throwing instruction within same try block
  }
  // We catch only MyException to be very specific
  catch (pcl::MyException& e)
  {
	  // Code to deal with the exception maybe changing myObject.the_parameters_used_
  }

  // Here we catch any exception
  #if 0
  catch (exception& e)
  {
	  // Code to deal with the exception maybe changing myObject.the_parameters_used_
  }
  #endif

Exceptions handling is really context dependent so there is no general
rule that can apply but here are some of the most used:

* exit with some error if the exception is critical
* modify the parameters for the function that threw the exception and
  recall it again
* throw an exception in your turn with meaningful message saying that
  you encountred an exception
* continue (really bad)
