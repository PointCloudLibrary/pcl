package com.itseez.bodyparts;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

class BodyPartLabelAdapter extends ArrayAdapter<BodyPartLabel> {
    private LayoutInflater inflater;

    public BodyPartLabelAdapter(Context context, int textViewResourceId, BodyPartLabel[] objects, LayoutInflater inflater) {
        super(context, textViewResourceId, objects);
        this.inflater = inflater;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        BodyPartLabel label = getItem(position);

        if (convertView == null) {
            convertView = inflater.inflate(R.layout.color_list_item, parent, false);
        }

        TextView color_name = (TextView) convertView.findViewById(R.id.color_name);
        color_name.setText(label.toString());

        ImageView color_picture = (ImageView) convertView.findViewById(R.id.color_picture);
        color_picture.setImageBitmap(label.icon);

        return convertView;
    }
}
