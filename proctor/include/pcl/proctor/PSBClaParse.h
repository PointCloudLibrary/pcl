
#ifndef __CLA_PARSE_H_
#define __CLA_PARSE_H_

/** The ClaParse files provide an interface for verifying and parsing .cla files.
    A fairly straighforward data structure is returned containing an array of categories.
*/


/**  A Category has a name, 
     a full name that includes its parent classes,
     the parent name,
     the number of models,
     and an array of integers for the model identifiers

     Copyright 2003 Princeton University
*/


#define MISC_CLASS "-1"

typedef struct{
  char *_name;
  char *_fullName;
  char** _models;
  int _numModels;
}PSBCategory;

/** A CategoryLIst has an array of Category pointers and the number of Categories.
    A double pointer is used for ease of qsort.
*/
typedef struct{
  PSBCategory** _categories;
  int _numCategories;
}PSBCategoryList;


/** Parses a .cla file and returns a CategoryList* data structure.
    If the .cla file has an incorrect format, a semi-helpful error
    message is printed, and the program exits.
*/
extern PSBCategoryList *parseFile(char *catFile, bool verbose);




#endif
