#ifndef WISEBAT_TOOLS_H
#define WISEBAT_TOOLS_H
#include <include/modelutils.h>

typedef struct _interpolation_points
{
    int n;
    double * x;
    double * y;

    //last index of the left point of the interpolation
    int lookup_idx;
} interpolation_points_t;



int get_param_interpolation_points(char * param, interpolation_points_t * points)
{
    switch(param[0])
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    {
        double y;
        if(get_param_double(param, &y))
        {
            return -1;
        }
        points->lookup_idx = 0;
        points->n = 1;
        points->x = malloc(sizeof(double));
        points->x[0] = 0;
        points->y = malloc(sizeof(double));
        points->y[0] = y;
    } break;
    case '[':
    {
        points->lookup_idx = 0;
        points->n = 1;
        int idx = 0;
        while(param[idx] != '\0')
        {
            if(param[idx] == ',')
            {
                ++points->n;
            }
            ++idx;
        }
        points->x = malloc(sizeof(double) * points->n);
        points->y = malloc(sizeof(double) * points->n);
        idx = 0;
        int char_idx = 0;
        while(param[char_idx] != '\0')
        {
            ++char_idx;
            sscanf(param + char_idx, "%lf:%lf", points->x + idx, points->y + idx);
            while(param[char_idx] != ']' && param[char_idx] != ',') ++char_idx;
            if(param[char_idx] == ']')
            {
                break;
            }
            ++idx;
        }
    } break;
    default:
    {
        //it's a filename
        FILE * file;
        if(!(file = fopen(param, "r")))
        {
            return -1;
        }
        while(fgetc(file) == '#')
        {
            while(fgetc(file) != '\n') { }
        }
        fseek(file, -1, SEEK_CUR);

        char s[200];
        int char_idx = 0;
        while((s[char_idx++] = fgetc(file)) != '\n') { }
        s[char_idx - 1] = '\0';
        sscanf(s, "%d", &(points->n));

        points->lookup_idx = 0;
        points->x = malloc(sizeof(double) * points->n);
        points->y = malloc(sizeof(double) * points->n);

        int idx = 0;
        //printf("n = %d\n", points->n);
        while(idx < points->n)
        {
            char_idx = 0;
            while((s[char_idx] = fgetc(file)) != '\n' && s[char_idx] != EOF) { ++char_idx; }
            s[char_idx] = '\0';
            sscanf(s, "%lf %lf", points->x + idx, points->y + idx);
            //printf("s = %s\n", s);
            //printf("%d %f %f\n", idx, *(points->x + idx), *(points->y + idx));
            ++idx;
        }
    }
    }
    return 0;
}

double linear_interpolation(interpolation_points_t * points, double x)
{
    if(points->n == 0)
    {
        return 0.0;
    }
    if(points->n == 1 || points->x[0] >= x)
    {
        return points->y[0];
    }
    int idx = points->lookup_idx + 1;
    if(idx > points->n)
    {
        idx = points->n;
    }
    while(idx > 1 && points->x[idx - 1] >= x) --idx;
    while(idx < points->n && points->x[idx] < x) ++idx;
    points->lookup_idx = idx;
    if(idx >= points->n)
    {
        return points->y[points->n-1];
    }

    double f = (points->y[idx] - points->y[idx - 1]) / (points->x[idx] - points->x[idx - 1]);
    double origin = points->y[idx] - f * points->x[idx];

    return f * x + origin;
}


#endif // WISEBAT_TOOLS_H

