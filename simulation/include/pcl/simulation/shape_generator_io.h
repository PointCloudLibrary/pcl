/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_SIMULATION_SHAPE_GENERATOR_IO_H_
#define PCL_SIMULATION_SHAPE_GENERATOR_IO_H_

#include <pcl/simulation/shape_generator.h>

#include <string>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace pcl
{
  namespace simulation
  {
    /** \brief Class for reading recipe files.
      * \author Markus Schoeler (mschoeler@gwdg.de)
      * \note The center of this shape will equal (0,0,0), regardless of the transformations applied in the recipe file. This is due to different shapes with different centers being combined.
      * \ingroup simulation
      */
    class RecipeFile : public MultiShape
    {
      public:
        typedef boost::shared_ptr<RecipeFile> Ptr;
        typedef std::vector<std::string> RecipeLinesT;
        typedef std::set<std::string> ParentRecipeNamesT;

        /** \brief Constructor for recipe files.
          * \param[in] recipe_file_name Filename of the recipe which should be processed.
          * \param[in] parent_recipe_names_set A set including the recipe filenames of the parents. If a recipe calls another recipe it should provide its own parent_recipe_names_set_ as an argument. If this is the first recipe, do not provide this argument.
          */
        RecipeFile (const std::string &recipe_file_name,
                    const ParentRecipeNamesT &parent_recipe_names_set = ParentRecipeNamesT ());

        /** \brief Check if the parsing was successful.
          * \returns True if parsing was successful.
          */
        inline bool
        parsedSuccessful () const
        {
          return (parsing_successful_);
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      private:
        /** \brief Path to recipe name. */
        boost::filesystem::path recipe_file_name_;

        /** \brief Stores the lines of a recipe. */
        RecipeLinesT recipe_lines_;

        /** \brief Recipes can call other recipes. This stores how deep we recursed down the recipes. Only used for formatting the terminal output.*/
        unsigned int recursion_depth_;

        /** \brief Prefix string to be used for intendenting the terminal output. */
        std::string prefix_;

        /** \brief This stores the current recipe filename as well as the filenames of all parent recipes.
          * \note If a new recipe is encountered we check if this has already been called for on of the parents. That way we detect infinite loops due to recipes calling on of it's parents recipes.
          */
        ParentRecipeNamesT parent_recipe_names_set_;

        /** \brief If no error occured this will be set to true. */
        bool parsing_successful_;

        /** \brief Static helper function to prepare/clean recipe lines (prune it of comments and excess spaces.
          * \param[in] line Line to clean
          * \returns The cleaned line.
          */
        static std::string
        cleanLine (std::string line);

        /** \brief Static helper function to convert a string to a float using boost.
          * \param[in] str String to convert.
          * \returns The corresponding float value.
          */
        inline static float
        toFloat (const std::string &str)
        {
          return (boost::lexical_cast<float> (str));
        }

        /** \brief Reads the recipe in recipe_file_name_.
          * \note Adds lines to recipe_lines_. If successful it will set parsing_successful_ to true.
          */
        void
        readRecipe ();

        /** \brief Reads the lines and adds all parsed shapes to shapes.
          * \param[in] lines This vector holds the recipe lines which should be processed.
          * \param[out] shapes The instructions found in lines will be translated into shapes.
          * \param[out] delete_overlap_flag If Delete_Overlap instruction is found in lines this variable will be set to that value. Otherwise this variable is true.
          * \param[out] label_handling If Label instructions are found in lines, this variable will be set. Otherwise it is Merge.
          * \returns True if the lines were parsed successfully. If an error occured (like unknown instructions) it will return false.
          * \note This is the main functionality for the class. It defines the instructions which can be translated by the recipe.
          */
        bool
        parseLines (const RecipeFile::RecipeLinesT& lines,
                    GeometricShapePtrVector& shapes,
                    bool& delete_overlap_flag,
                    MultiShape::LabelHandling &label_handling);

        /** \brief Helper function to treat blocks encountered in a recipe (like Multi_begin, Cut_begin ...). 
          * \param[in,out] itr Line Iterator which needs to be defined in the caller function. Since blocks span multiple lines this functions iterates itr as well.
          * \param[in] end_itr Iterator pointing to the last line of the instructions currently parsed. Is required since we increment itr in this function.
          * \param[in] delimiter A vector of strings denoting the delimiters of the block.
          * \param[out] blocks The blocks which were found excluding the delimiters. The number of blocks is equal to the number of delimeters provided.
          * \returns Function returns true processing the block was successful. If a delimeter was not found or if we reached the end of the instructions this will return false.
          */
        bool
        parseBlock (RecipeLinesT::const_iterator &itr,
                    const RecipeLinesT::const_iterator end_itr,
                    const RecipeLinesT delimiter,
                    std::vector<RecipeLinesT> &blocks);

    };
  }  // End namespace simulation
}  // End namespace pcl

#endif // PCL_SIMULATION_SHAPE_GENERATOR_IO_H_
