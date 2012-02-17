// Philip Shilane
// Copyright 2003 Princeton University 
//
// This utility verifies the format of a .cla file.
// A valid cla file has:
// PSB FormatNum
// numCategories numModels
// category parentCategory numModels
// one model id per line
//
// Where parentCategory and numModels can be 0 (zero).
// All top level categories should specify a parent category of 0 (zero).
// Categories and numbers cannot have white space within the name.
// Parent categories must be defined before children categories, 0 is already defined.
// All category names must be unique.
// Arbitrary whitespace is otherwise allowed. 
// The .cla file should be in text format and can have up to 128 characters per line.
//
// This parser only handles PSB format 1.
//

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <assert.h>

#include "PSBClaParse.h"

#define CURR_FORMAT 2
#define PSB "PSB"
#define BASE_CATEGORY "0"
/*
typedef char bool;
#define true 1
#define false 0
*/

static void error(FILE *file, const char *format, ...);
static void createFullName(char *catNameBuffer, const char *className, 
                           const char *parentClassName, PSBCategoryList *categoryList, 
                           int maxNumCategories);
static PSBCategory *defineCategory(const char *category, const char* parent, int numModels);
static bool isCategoryDefined(PSBCategoryList *categoryList, const char *category, 
                              int currentNumCategories);




extern PSBCategoryList *parseFile(char *catFile, bool verbose){
  FILE *file;
  char firstBuffer[256], secondBuffer[256], catNameBuffer[256], modelBuffer[256];
  int num, numCategories, currentNumCategories, numFound, i;
  int numModels, currNumModels, currNumTotalModels, numTotalModels;
  PSBCategoryList *categoryList;

  file = fopen(catFile, "r");
  if (file == NULL){
    fprintf(stderr, "trouble opening %s\n", catFile);
    exit(1);
  }

  // check header
  if(fscanf(file, "%s %d", firstBuffer, &num) != 2 || strcmp(PSB, firstBuffer) || num > CURR_FORMAT){
    error(file, "%s is not a PSB file of format %d\n", catFile, CURR_FORMAT);
  }

  // get num categories
  if (fscanf(file, "%d %d", &numCategories, &numTotalModels) != 2){
    error(file, "%s does not list the number of categories or models\n", catFile);
  }

  if (numCategories < 0 || numTotalModels < 0){
    error(file, "%d categories and %d models is invalid\n", numCategories, numTotalModels);
  }
  

  categoryList = (PSBCategoryList*)malloc(sizeof(PSBCategoryList));
  assert(categoryList != NULL);
  categoryList->_numCategories = numCategories;
  categoryList->_categories = (PSBCategory **)malloc(numCategories * sizeof(PSBCategory*));
  assert(categoryList != NULL);

  currentNumCategories = 0;
  currNumTotalModels = 0;

  while(1){
    numFound = fscanf(file, "%s %s %d", firstBuffer, secondBuffer, &numModels);
    
    if (numFound == 0 || numFound == EOF){
      break; // end of file
    }else if (numFound == 1){
      error(file, "poorly formated category line, %s\n", firstBuffer);
    }else if (numFound == 2){
      error(file, "poorly formated category line, %s %s\n", firstBuffer, secondBuffer);
    }else if (numFound > 3){
      error(file, "poorly formated category line\n");
    }

    if (numModels < 0){
      error(file, "%d is an invalid number of categories\n", numModels);
    }

    if (isCategoryDefined(categoryList, firstBuffer, currentNumCategories)){
      error(file, "%s is already a defined category\n", firstBuffer);
    }

    if (!isCategoryDefined(categoryList, secondBuffer, currentNumCategories)){
      error(file, "%s is not a defined category\n", secondBuffer);
    }


    createFullName(catNameBuffer, firstBuffer, secondBuffer, categoryList, currentNumCategories);

    categoryList->_categories[currentNumCategories] = defineCategory(firstBuffer, catNameBuffer, numModels);

    if (verbose) printf("processing category %s, ", firstBuffer);

    currNumModels = 0;
    
    for(i = 0; i < numModels; ++i){
      numFound = fscanf(file,"%s", modelBuffer);
      if (numFound != 1){
        error(file, "%s has an incorrect number of models, read %d, should be %d\n", firstBuffer, currNumModels, numModels);
      }
      categoryList->_categories[currentNumCategories]->_models[currNumModels] = strdup(modelBuffer);
      currNumModels++;
      currNumTotalModels++;
    }
    if (verbose) printf("finished.\n");
    ++currentNumCategories;

  }
  
  if (currentNumCategories != numCategories){
    error(file, "expected %d categories, found %d\n", numCategories, currentNumCategories);
  }

  if (currNumTotalModels != numTotalModels){
    error(file, "expected %d models, found %d\n", numTotalModels, currNumTotalModels);
  }

  
  if (fclose(file) == EOF){
    fprintf(stderr, "trouble closing %s\n", catFile);
  }

  if (verbose) printf("Validated %s, %d categories, %d models\n", catFile, numCategories, currNumTotalModels);
  return categoryList;
}


static void error(FILE *file, const char *format, ...){
  
  char buffer[256];
  va_list ap;

  va_start(ap, format);
  vsprintf(buffer, format, ap);
  va_end(ap);

  fprintf(stderr, buffer);

  fclose(file);

  exit(1);
  
 
}


static PSBCategory* defineCategory(const char *categoryName, const char *fullName, int numModels){
  PSBCategory * category;

  category = (PSBCategory*)malloc(sizeof(PSBCategory));
  assert(category!=NULL);

  category->_models = (char**)malloc(numModels * sizeof(char*));
  assert(category->_models != NULL);
  memset(category->_models, 0, numModels * sizeof(char*));

  category->_name = strdup(categoryName);
  assert(category->_name != NULL);
  category->_fullName = strdup(fullName);
  assert(category->_fullName != NULL);

  category->_numModels = numModels;
  return category;
}

static bool isCategoryDefined(PSBCategoryList *categoryList, const char *category, int currentNumCategories){
  
  int i;
  
  if (strcmp(category, BASE_CATEGORY) == 0){
    return true;
  }

  for(i = 0; i < currentNumCategories; ++i){
    if (strcmp(category, categoryList->_categories[i]->_name)==0) return true;
  }
  
  return false;
}

static void createFullName(char *catNameBuffer, const char *className, 
                           const char *parentClassName, PSBCategoryList *categoryList, 
                           int maxNumCategories){
  
  int j;
  
  if (strcmp(parentClassName, BASE_CATEGORY) == 0){
    strcpy(catNameBuffer, className);
  }else{
    for(j = 0; j < maxNumCategories; ++j){
      if (strcmp(categoryList->_categories[j]->_name, parentClassName)==0){
        sprintf(catNameBuffer, "%s___%s", categoryList->_categories[j]->_fullName, className);
        return;
      }
    }
    assert(0);
  }
  
}


