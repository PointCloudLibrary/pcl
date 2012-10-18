#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include <boost/cstdint.hpp>

typedef boost::uint8_t Label;

struct Offsets
{
    boost::int16_t du1, dv1, du2, dv2;
};

struct Node
{
  Offsets offsets;
  boost::int16_t threshold;
};

int
main(int argc, char * argv[])
{
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " tree.txt tree.bin\n";
    return (EXIT_FAILURE);
  }

  std::ifstream txt_file (argv[1]);

  if (!txt_file)
  {
    std::cerr << "Couldn't open " << argv[1] << ".\n";
    return (EXIT_FAILURE);
  }

  std::ofstream bin_file (argv[2], std::ios::out | std::ios::binary);

  if (!bin_file)
  {
    std::cerr << "Couldn't open " << argv[2] << ".\n";
    return (EXIT_FAILURE);
  }

  boost::uint16_t depth;
  txt_file >> depth;

  if (!txt_file)
  {
    std::cerr << "Malformed input file.\n";
    return (EXIT_FAILURE);
  }

  bin_file.write(reinterpret_cast<char *> (&depth), sizeof depth);

  unsigned num_nodes  = (1 << depth) - 1;
  unsigned num_leaves = (1 << depth);

  std::vector<Node> nodes (num_nodes);
  std::vector<Label> leaves (num_leaves);

  for (unsigned i = 0; i < num_nodes; ++i)
  {
    txt_file >> nodes[i].offsets.du1;
    txt_file >> nodes[i].offsets.dv1;
    txt_file >> nodes[i].offsets.du2;
    txt_file >> nodes[i].offsets.dv2;
    txt_file >> nodes[i].threshold;
  }

  bin_file.write (reinterpret_cast<char *> (&nodes.front()), num_nodes * sizeof (Node));

  for (unsigned i = 0; i < num_leaves; ++i)
  {
    int label;
    txt_file >> label;
    leaves[i] = label;
  }

  bin_file.write (reinterpret_cast<char *> (&leaves.front()), num_leaves * sizeof (Label));

  if (!txt_file)
  {
    std::cerr << "Malformed input file or read error.\n";
    return (EXIT_FAILURE);
  }

  if (!bin_file)
  {
    std::cerr << "Write error.\n";
    return (EXIT_FAILURE);
  }

  return (EXIT_SUCCESS);
}
