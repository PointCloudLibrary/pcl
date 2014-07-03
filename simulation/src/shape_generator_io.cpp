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

#include <pcl/simulation/shape_generator_io.h>

pcl::simulation::RecipeFile::RecipeFile (const std::string &recipe_file_name,
                                         const ParentRecipeNamesT &parent_recipe_names_set) :
  recipe_file_name_ (recipe_file_name),
  parent_recipe_names_set_ (parent_recipe_names_set)
{
  parent_recipe_names_set_.insert (recipe_file_name);
  recursion_depth_ = parent_recipe_names_set_.size ();
  prefix_ = std::string (recursion_depth_ * 2, ' ');
  readRecipe ();
}

std::string
pcl::simulation::RecipeFile::cleanLine (std::string line)
{
  RecipeLinesT parsed_line;
  boost::algorithm::trim_if (line, boost::algorithm::is_any_of ("\t "));
  // ignore hash-tags (for comments)
  std::size_t hash_pos = line.find_first_of ("#");
  if (hash_pos != std::string::npos)
    line.erase (hash_pos);
  return (line);
}

bool
pcl::simulation::RecipeFile::parseBlock (RecipeLinesT::const_iterator &itr,
                                         const RecipeLinesT::const_iterator end_itr,
                                         const RecipeLinesT delimiter,
                                         std::vector<RecipeLinesT> &blocks)
{
  blocks.clear ();
  // first delimiter is reserved for the block opening string, thus size - 1
  blocks.resize (delimiter.size () - 1);
  // this treats the case of nested blocks.
  unsigned int nested_blocks_counter = 0;
  // first delimiter is reserved for the block opening string.

  std::string opening_delimiter = delimiter[0];
  std::string closing_delimiter = delimiter[delimiter.size () - 1];

  unsigned int delimiter_counter = 1;
  while (delimiter_counter != delimiter.size ())
  {
    bool delimiter_found = false;
    while (!delimiter_found)
    {
      PCL_INFO ("%sRead lines within block (next delimiter %s). Current line: %s\n", prefix_.c_str (), delimiter[delimiter_counter].c_str (), itr->c_str ());

      // check if we have nested blocks if so, we need to treat delimiters which would normally trigger the transition between blocks as part of the block.
      bool ignore_delimiters;
      if (*itr == opening_delimiter)
      {
        ++nested_blocks_counter;
      }

      if (nested_blocks_counter > 0)
      {
        PCL_INFO ("%sCurrently within nested block, ignoring delimiters\n", prefix_.c_str ());
        ignore_delimiters = true;
        if (*itr == closing_delimiter)
          --nested_blocks_counter;
      }
      else
      {
//         PCL_INFO ("%sCurrently NOT within nested block\n");
        ignore_delimiters = false;
      }

      if (*itr == delimiter[delimiter_counter] && !ignore_delimiters)
      {
        delimiter_found = true;
        ++itr;
        ++delimiter_counter;
      }
      else
      {
        blocks[delimiter_counter - 1].push_back (*itr);
        ++itr;
        if (itr == end_itr)
        {
          PCL_WARN ("%sCould not find %s\n", prefix_.c_str (), delimiter[delimiter_counter].c_str ());
          return (false);
        }
      }
    }
  }
  // the iterator points now to the line after the last delimiter, but needs to point to the last line of the block (RecipeFile::parseLines increments for itself). Thus decrement.
  --itr;
  return (true);
}

bool
pcl::simulation::RecipeFile::parseLines (const RecipeLinesT& lines,
                                         GeometricShapePtrVector &shapes,
                                         bool &delete_overlap_flag,
                                         MultiShape::LabelHandling &label_handling)
{

  shapes.clear ();
  delete_overlap_flag = true;
  label_handling = Merge;
  for (RecipeLinesT::const_iterator itr = lines.begin (); itr != lines.end (); ++itr)
  {
    PCL_INFO ("%sParsing Line: %s\n", prefix_.c_str (), itr->c_str ());

    // ignore empty lines
    if (itr->size () == 0)
    {
      continue;
    }

    RecipeLinesT substrings;
    boost::algorithm::split (substrings, *itr, boost::algorithm::is_any_of ("\t "), boost::algorithm::token_compress_on);

    // Cylinder
    if (substrings[0] == "Cylinder" && substrings.size () == 3)
    {
      shapes.push_back (Cylinder::Ptr (new Cylinder (toFloat (substrings[1]), toFloat (substrings[2]))));
      continue;
    }

    // Cuboid
    if (substrings[0] == "Cuboid" && substrings.size () == 4)
    {
      shapes.push_back (Cuboid::Ptr (new Cuboid (toFloat (substrings[1]), toFloat (substrings[2]), toFloat (substrings[3]))));
      continue;
    }

    // Sphere
    if (substrings[0] == "Sphere" && substrings.size () == 2)
    {
      shapes.push_back (Sphere::Ptr (new Sphere (toFloat (substrings[1]))));
      continue;
    }

    // Cone
    if (substrings[0] == "Cone" && substrings.size () == 3)
    {
      shapes.push_back (Cone::Ptr (new Cone (toFloat (substrings[1]), toFloat (substrings[2]))));
      continue;
    }

    // Torus (full or part)
    if (substrings[0] == "Torus" && (substrings.size () == 3 || substrings.size () == 5))
    {
      if (substrings.size () == 3)
        shapes.push_back (Torus::Ptr (new Torus (toFloat (substrings[1]), toFloat (substrings[2]))));
      else
        shapes.push_back (
            Torus::Ptr (new Torus (toFloat (substrings[1]), toFloat (substrings[2]), toFloat (substrings[3]) * M_PI / 180.f, toFloat (substrings[4]) * M_PI / 180.f)));
      continue;
    }

    // Wedge
    if (substrings[0] == "Wedge" && substrings.size () == 6)
    {
      shapes.push_back (
          Wedge::Ptr (new Wedge (toFloat (substrings[1]), toFloat (substrings[2]), toFloat (substrings[3]), toFloat (substrings[4]), toFloat (substrings[5]))));
      continue;
    }

    // Transformations
    if (substrings[0] == "T" && substrings.size () == 4)
    {
      if (shapes.size () == 0)
      {
        PCL_WARN ("No shapes created yet, ignoring transformation\n");
        continue;
      }
      shapes.back ()->translate (toFloat (substrings[1]), toFloat (substrings[2]), toFloat (substrings[3]));
      continue;
    }
    if ((substrings[0] == "R" || substrings[0] == "RC") &&
        (substrings.size () == 3 || substrings.size () == 5))
    {
      if (shapes.size () == 0)
      {
        PCL_WARN ("No shapes created yet, ignoring transformation\n");
        continue;
      }
      Eigen::Vector3f axis;
      bool rotate_in_place = (substrings[0] == "R");
      if (substrings[1] == "x" || substrings[1] == "y" || substrings[1] == "z")
      {
        if (substrings[1] == "x")
          axis = Eigen::Vector3f (1, 0, 0);
        else if (substrings[1] == "y")
          axis = Eigen::Vector3f (0, 1, 0);
        else if (substrings[1] == "z")
          axis = Eigen::Vector3f (0, 0, 1);
        shapes.back ()->rotate (axis, toFloat (substrings[2]), rotate_in_place);
      }
      else
      {
        axis = Eigen::Vector3f (toFloat (substrings[1]), toFloat (substrings[2]), toFloat (substrings[3]));
        shapes.back ()->rotate (axis, toFloat (substrings[4]));
      }

      continue;
    }

    // Multi Shape Object
    if (substrings[0] == "Multi_begin" && substrings.size () == 1)
    {
      ++itr;
      std::vector<RecipeLinesT> blocks;
      RecipeLinesT delimiter;
      delimiter.push_back ("Multi_begin");
      delimiter.push_back ("Multi_end");
      // check if the block was parsed sucessfully, before processing the lines
      if (parseBlock (itr, lines.end (), delimiter, blocks))
      {
        GeometricShapePtrVector shapes_from_block_ptr;
        bool overlap_flag_from_block;
        LabelHandling label_handling_from_block;
        // check if the lines were parsed sucessfully, before adding the new object to the vector
        if (parseLines (blocks[0], shapes_from_block_ptr, overlap_flag_from_block, label_handling_from_block))
          shapes.push_back (MultiShape::Ptr (new MultiShape (shapes_from_block_ptr, overlap_flag_from_block, label_handling_from_block)));
        else
          PCL_WARN ("%sError while reading the multi block. Skipping it.\n", prefix_.c_str ());
      }
      else
      {
        PCL_WARN ("%sCould not find Multi_end. Skipping the rest.\n", prefix_.c_str ());
        return (false);
      }
      continue;
    }

    // Cutshape Object
    if (substrings[0] == "Cut_begin" && substrings.size () == 1)
    {
      ++itr;
      std::vector<RecipeLinesT> blocks;
      RecipeLinesT delimiter;
      delimiter.push_back ("Cut_begin");
      delimiter.push_back ("by");
      delimiter.push_back ("Cut_end");
      // check if the block was parsed sucessfully, before processing the lines
      if (parseBlock (itr, lines.end (), delimiter, blocks))
      {
        GeometricShapePtrVector shapes_from_cut_block_ptr, shapes_from_cutter_block_ptr;
        bool overlap_flag_from_cut_block, overlap_flag_from_cutter_block;
        LabelHandling label_handling_from_cut_block, label_handling_from_cutter_block;
        // check if the lines were parsed sucessfully, before adding the new object to the vector
        if (parseLines (blocks[0], shapes_from_cut_block_ptr, overlap_flag_from_cut_block, label_handling_from_cut_block)
            && parseLines (blocks[1], shapes_from_cutter_block_ptr, overlap_flag_from_cutter_block, label_handling_from_cutter_block))
        {
          MultiShape::Ptr ToCut (new MultiShape (shapes_from_cut_block_ptr, overlap_flag_from_cut_block, Single));
          MultiShape::Ptr Cutter (new MultiShape (shapes_from_cutter_block_ptr, overlap_flag_from_cutter_block, Single));
          shapes.push_back (CutShape::Ptr (new CutShape (ToCut, Cutter)));
        }
        else
          PCL_WARN ("%sError while reading the cut block. Skipping it.\n", prefix_.c_str ());
      }
      else
      {
        PCL_WARN ("%sCould not find the delimiters of the cut block. Skipping the rest.\n", prefix_.c_str ());
        return (false);
      }
      continue;
    }

    // Recipe Object
    if (substrings[0] == "Recipe" && substrings.size () == 2)
    {
      // check if the recipe filename has been processed for a predecessor. If that is the case we would create an infinite loop.
      if (parent_recipe_names_set_.find (substrings[1]) == parent_recipe_names_set_.end ())
      {
        RecipeFile::Ptr new_recipe (new RecipeFile (substrings[1], parent_recipe_names_set_));
        // check if the parsing was successful before adding the new object to the vector
        if (new_recipe->parsedSuccessful ())
          shapes.push_back (new_recipe);
      }
      else
        PCL_WARN ("%sRecursive loop detected when trying to parse recipe %s. Skipping this line.\n", prefix_.c_str (), substrings[1].c_str ());
      continue;
    }

    // Lines for setting options
    if (substrings[0] == "Delete_overlap" && substrings.size () == 2)
    {
      if (substrings[1] == "True")
        delete_overlap_flag = true;
      else if (substrings[1] == "False")
        delete_overlap_flag = false;
      else
        PCL_WARN ("%sDelete_overlap only takes True or False as options. Found %s. Skipping this line.\n", prefix_.c_str (), substrings[1].c_str ());
      continue;
    }

    if (substrings[0] == "Label" && substrings.size () == 2)
    {
      if (substrings[1] == "Single")
        label_handling = Single;
      else if (substrings[1] == "Merge")
        label_handling = Merge;
      else if (substrings[1] == "Append")
        label_handling = Append;
      else if (substrings[1] == "Preserve")
        label_handling = Preserve;
      else
        PCL_WARN ("%s%s is not a valid label handling instruction. Valid strings: Single, Merge, Append, Preserve. Skipping this line.\n", prefix_.c_str (), substrings[1].c_str ());
      continue;
    }

    PCL_WARN ("%sCould not parse line: %s. Skipping this line.\n", prefix_.c_str (), itr->c_str ());
  }
  return (shapes.size () > 0);
}

void
pcl::simulation::RecipeFile::readRecipe ()
{
  PCL_INFO ("%sOpening file %s\n", prefix_.c_str (), recipe_file_name_.c_str ());
  boost::filesystem::ifstream recipe_file (recipe_file_name_);
  if (!recipe_file.is_open ())
  {
    PCL_WARN ("%sCould not open recipe file %s.\n", prefix_.c_str (), recipe_file_name_.c_str ());
    parsing_successful_ = false;
    return;
  }

  recipe_lines_.clear ();
  std::string line;
  // clean and add lines to the memory.
  while (std::getline (recipe_file, line))
  {
    line = cleanLine (line);
    if (!line.empty ())
      recipe_lines_.push_back (line);
  }
  parsing_successful_ = parseLines (recipe_lines_, shapes_ptrs_, delete_overlap_, label_handling_);
  PCL_INFO ("%sClosing file %s\n", prefix_.c_str (), recipe_file_name_.c_str ());
}
