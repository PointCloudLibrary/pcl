#ifndef PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_
#define PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_

#include <string>
#include <vector>
#include <iosfwd>
#include <boost/shared_ptr.hpp>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      class PersonAttribs
      {
      public:
        typedef boost::shared_ptr<PersonAttribs> Ptr;

          /** \brief Read XML configuration file for a specific person */
        void 
        readPersonXMLConfig (std::istream& is);

        /** \brief Write XML configuration file for a specific person */
        void 
        writePersonXMLConfig (std::ostream& os);

      private:
        std::string                                 name_;                  // Name of the person
        std::vector<float>                          max_part_size_;         // Max primary eigenvalue for each body part
        std::vector<std::vector<float> >            part_ideal_length_;     // Ideal length between two body parts
        std::vector<std::vector<float> >            max_length_offset_;     // Max allowed length offset between two body parts
      };
    }
  }
}

#endif /* PLC_GPU_PEOPLE_PERSON_ATTRIBS_H_ */