#ifndef PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_
#define PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_

#include <string>
#include <vector>
#include <iosfwd>
#include <boost/shared_ptr.hpp>

#include <pcl/pcl_exports.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      class PCL_EXPORTS PersonAttribs
      {
        public:
          typedef boost::shared_ptr<PersonAttribs> Ptr;

          /** \brief Constructor creates generic values from **/
          PersonAttribs();

          /**
           * \brief Read XML configuration file for a specific person
           * \param[in] is input stream of file
           * \return 0 when successfull, -1 when an error occured, datastructure might become corrupted in the process
           **/
          int
          readPersonXMLConfig (std::istream& is);

          /**
           * \brief Write XML configuration file for a specific person
           * \param[in] os output stream of file, extension determines format
           **/
          void
          writePersonXMLConfig (std::ostream& os);

          std::string                                 name_;                  // Name of the person
          std::vector<float>                          max_part_size_;         // Max primary eigenvalue for each body part
          std::vector<std::vector<float> >            part_ideal_length_;     // Ideal length between two body parts
          std::vector<std::vector<float> >            max_length_offset_;     // Max allowed length offset between two body parts
          std::vector<int>                            nr_of_children_;        // The number of children for each part
      };
    }
  }
}

#endif /* PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_ */
