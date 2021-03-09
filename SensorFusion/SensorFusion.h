int fusion(double x, double y, double z. double g_x, double g_y, double g_z){
  double t_x, t_y, t_z;
  t_x = t_x + x*cos(g_x);
  t_z = t_z + x*sin(g_x);
  t_y = t_y + y*cos(g_y);
  t_z = t_z + y*sin(g_y);
  t_x = t_x + z*sin(g_z); // note there is some ambiguity here, this could be either x or y based on orientation, not sure how to figure out which
  t_z = t_z + z*sin(g_z);
}
