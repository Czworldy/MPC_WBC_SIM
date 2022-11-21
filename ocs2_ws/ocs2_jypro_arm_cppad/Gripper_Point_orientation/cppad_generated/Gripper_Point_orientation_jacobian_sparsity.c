void Gripper_Point_orientation_jacobian_sparsity(unsigned long const** row,
                                                 unsigned long const** col,
                                                 unsigned long* nnz) {
   static unsigned long const rows[27] = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[27] = {9,10,11,24,25,26,27,28,29,9,10,11,24,25,26,27,28,29,9,10,11,24,25,26,27,28,29};
   *row = rows;
   *col = cols;
   *nnz = 27;
}
