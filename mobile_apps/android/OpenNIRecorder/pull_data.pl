$begin = 1;
$end = 3000;
for ($count = $begin; $count <= $end; $count++) {
	$my_c = sprintf( "%06d", $count );
	$rgb_cmd = "adb pull /mnt/sdcard2/pcl/out_rgb_$my_c.png";
	$depth_cmd = "adb pull /mnt/sdcard2/pcl/out_depth_$my_c.png";
	`$rgb_cmd`;
	`$depth_cmd`;
}
