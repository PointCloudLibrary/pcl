#include <pcl/apps/cloud_composer/cloud_browser.h>
#include <pcl/apps/cloud_composer/project_model.h>


pcl::cloud_composer::CloudBrowser::CloudBrowser (QWidget* parent)
  : QTreeView (parent)
{
   this->setItemDelegate (new BackgroundDelegate (this));
}


void
pcl::cloud_composer::CloudBrowser::setModel (QAbstractItemModel* new_model)
{
  QTreeView::setModel (new_model);
  
  current_project_model_ = dynamic_cast <ProjectModel*>(new_model);
}



void
pcl::cloud_composer::BackgroundDelegate::paint (QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  // Fill the background before calling the base class paint
  // otherwise selected cells would have a white background
 // QVariant background = index.data (Qt::BackgroundRole);
 // if (background.canConvert<QBrush> ())
 //   painter->fillRect (option.rect, background.value<QBrush> ());

  QVariant text_color_variant = index.data (Qt::ForegroundRole);
  if (text_color_variant.canConvert<QColor> ())
  {
    QColor text_color = text_color_variant.value<QColor> ();
    QStyleOptionViewItem option_copy = option;
    option_copy.palette.setColor (QPalette::HighlightedText, text_color);
    QStyledItemDelegate::paint (painter, option_copy, index);
  }
  else
    QStyledItemDelegate::paint (painter, option, index);
  
  
}
