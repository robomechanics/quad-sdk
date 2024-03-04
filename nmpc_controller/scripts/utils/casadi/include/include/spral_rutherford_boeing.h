#ifndef SPRAL_RUTHERFORD_BOEING_H
#define SPRAL_RUTHERFORD_BOEING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct spral_rb_read_options {
   int array_base;
   bool add_diagonal;
   float extra_space;
   int lwr_upr_full;
   int values;
};

struct spral_rb_write_options {
   int array_base;
   char val_format[21];
};

void spral_rb_default_read_options(struct spral_rb_read_options *options);
void spral_rb_default_write_options(struct spral_rb_write_options *options);
int spral_rb_peek(const char *filename, int *m, int *n, int64_t *nelt, int64_t *nvar,
      int64_t *nval, enum spral_matrix_type *matrix_type, char *type_code,
      char *title, char *identifier);
int spral_rb_read(const char *filename, void **handle,
      enum spral_matrix_type *matrix_type, int *m, int *n, int64_t **ptr,
      int **row, double **val, const struct spral_rb_read_options *options,
      char *title, char *identifier, int *state);
int spral_rb_read_ptr32(const char *filename, void **handle,
      enum spral_matrix_type *matrix_type, int *m, int *n, int **ptr,
      int **row, double **val, const struct spral_rb_read_options *options,
      char *title, char *identifier, int *state);
int spral_rb_write(const char *filename, enum spral_matrix_type matrix_type,
      int m, int n, const int64_t *ptr, const int *row, const double * val,
      const struct spral_rb_write_options *options, const char *title,
      const char *identifier);
int spral_rb_write_ptr32(const char *filename,
      enum spral_matrix_type matrix_type,
      int m, int n, const int *ptr, const int *row, const double * val,
      const struct spral_rb_write_options *options, const char *title,
      const char *identifier);
void spral_rb_free_handle(void **handle);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_RUTHERFORD_BOEING_H
