#include <pcl/apps/cloud_composer/cloud_composer.h>

/////////// MAIN ////////////////////
int
main (int argc, char ** argv)
{
  // Initialize QT
  QApplication app (argc, argv);
  
  pcl::cloud_composer::ComposerMainWindow cc;
  cc.show ();
  return (app.exec ());
}