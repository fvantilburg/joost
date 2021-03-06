// Copyright (C) 2012  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.
#undef DLIB_INTERPOlATION_ABSTRACT_
#ifdef DLIB_INTERPOlATION_ABSTRACT_ 

#include "../pixel.h"

namespace dlib
{

// ----------------------------------------------------------------------------------------

    class interpolate_nearest_neighbor
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This object is a tool for performing nearest neighbor interpolation
                on an image.  
        !*/

    public:

        template <
            typename image_type, 
            typename pixel_type
            >
        bool operator() (
            const image_type& img,
            const dlib::point& p,
            pixel_type& result
        ) const;
        /*!
            requires
                - image_type == is an implementation of array2d/array2d_kernel_abstract.h
                - pixel_traits<typename image_type::type>::has_alpha == false
                - pixel_traits<pixel_type> is defined
            ensures
                - if (p is located inside img) then
                    - #result == img[p.y()][p.x()]
                      (This assignment is done using assign_pixel(#result, img[p.y()][p.x()]), 
                      therefore any necessary color space conversion will be performed)
                    - returns true
                - else
                    - returns false
        !*/

    };

// ----------------------------------------------------------------------------------------

    class interpolate_bilinear
    {

        /*!
            WHAT THIS OBJECT REPRESENTS
                This object is a tool for performing bilinear interpolation
                on an image.  This is performed by looking at the 4 pixels
                nearest to a point and deriving an interpolated value from them.
        !*/

    public:

        template <
            typename T, 
            typename image_type,
            typename pixel_type
            >
        bool operator() (
            const image_type& img,
            const dlib::vector<T,2>& p,
            pixel_type& result
        ) const;
        /*!
            requires
                - image_type == is an implementation of array2d/array2d_kernel_abstract.h
                - pixel_traits<typename image_type::type>::has_alpha == false
                - pixel_traits<pixel_type> is defined
            ensures
                - if (there is an interpolatable image location at point p in img) then
                    - #result == the interpolated pixel value from img at point p.
                    - assign_pixel() will be used to write to #result, therefore any
                      necessary color space conversion will be performed.
                    - returns true
                    - if img contains RGB pixels then the interpolation will be in color.
                      Otherwise, the interpolation will be performed in a grayscale mode.
                - else
                    - returns false
        !*/
    };

// ----------------------------------------------------------------------------------------

    class interpolate_quadratic
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This object is a tool for performing quadratic interpolation
                on an image.  This is performed by looking at the 9 pixels
                nearest to a point and deriving an interpolated value from them.
        !*/

    public:

        template <
            typename T, 
            typename image_type,
            typename pixel_type
            >
        bool operator() (
            const image_type& img,
            const dlib::vector<T,2>& p,
            pixel_type& result
        ) const;
        /*!
            requires
                - image_type == is an implementation of array2d/array2d_kernel_abstract.h
                - pixel_traits<typename image_type::type>::has_alpha == false
                - pixel_traits<pixel_type> is defined
            ensures
                - if (there is an interpolatable image location at point p in img) then
                    - #result == the interpolated pixel value from img at point p
                    - assign_pixel() will be used to write to #result, therefore any
                      necessary color space conversion will be performed.
                    - returns true
                    - if img contains RGB pixels then the interpolation will be in color.
                      Otherwise, the interpolation will be performed in a grayscale mode.
                - else
                    - returns false
        !*/
    };

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    class black_background
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This is a function object which simply sets a pixel 
                to have a black value.
        !*/

    public:
        template <typename pixel_type>
        void operator() ( pixel_type& p) const { assign_pixel(p, 0); }
    };

// ----------------------------------------------------------------------------------------

    class white_background
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This is a function object which simply sets a pixel 
                to have a white value.
        !*/

    public:
        template <typename pixel_type>
        void operator() ( pixel_type& p) const { assign_pixel(p, 255); }
    };

// ----------------------------------------------------------------------------------------

    class no_background
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This is a function object which does nothing.  It is useful
                when used with the transform_image() routine defined below
                if no modification of uninterpolated output pixels is desired.
        !*/
    public:
        template <typename pixel_type>
        void operator() ( pixel_type& ) const { }
    };

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename interpolation_type,
        typename point_mapping_type,
        typename background_type
        >
    void transform_image (
        const image_type1& in_img,
        image_type2& out_img,
        const interpolation_type& interp,
        const point_mapping_type& map_point,
        const background_type& set_background,
        const rectangle& area
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - map_point should be a function which takes dlib::vector<T,2> objects and
              returns dlib::vector<T,2> objects.  An example is point_transform_affine.
            - set_background should be a function which can take a single argument of
              type image_type2::type.  Examples are black_background, white_background,
              and no_background.
            - get_rect(out_img).contains(area) == true
            - is_same_object(in_img, out_img) == false
        ensures
            - The map_point function defines a mapping from pixels in out_img to pixels
              in in_img.  transform_image() uses this mapping, along with the supplied
              interpolation routine interp, to fill the region of out_img defined by
              area with an interpolated copy of in_img.  
            - This function does not change the size of out_img.
            - Only pixels inside the region defined by area in out_img are modified.
            - For all locations r and c such that area.contains(c,r) but have no corresponding 
              locations in in_img:
                - set_background(out_img[r][c]) is invoked
                  (i.e. some parts of out_img might correspond to areas outside in_img and
                  therefore can't supply interpolated values.  In these cases, these
                  pixels can be assigned a value by the supplied set_background() routine)
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename interpolation_type,
        typename point_mapping_type,
        typename background_type
        >
    void transform_image (
        const image_type1& in_img,
        image_type2& out_img,
        const interpolation_type& interp,
        const point_mapping_type& map_point,
        const background_type& set_background
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - map_point should be a function which takes dlib::vector<T,2> objects and
              returns dlib::vector<T,2> objects.  An example is point_transform_affine.
            - set_background should be a function which can take a single argument of
              type image_type2::type.  Examples are black_background, white_background,
              and no_background.
            - is_same_object(in_img, out_img) == false
        ensures
            - performs: 
              transform_image(in_img, out_img, interp, map_point, set_background, get_rect(out_img));
              (i.e. runs transform_image() on the entire out_img)
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename interpolation_type,
        typename point_mapping_type
        >
    void transform_image (
        const image_type1& in_img,
        image_type2& out_img,
        const interpolation_type& interp,
        const point_mapping_type& map_point
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - map_point should be a function which takes dlib::vector<T,2> objects and
              returns dlib::vector<T,2> objects.  An example is point_transform_affine.
            - is_same_object(in_img, out_img) == false
        ensures
            - performs: 
              transform_image(in_img, out_img, interp, map_point, black_background(), get_rect(out_img));
              (i.e. runs transform_image() on the entire out_img and sets non-interpolated
              pixels to black)
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename interpolation_type
        >
    point_transform_affine rotate_image (
        const image_type1& in_img,
        image_type2& out_img,
        double angle,
        const interpolation_type& interp
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img == a copy of in_img which has been rotated angle radians counter clockwise.
              The rotation is performed with respect to the center of the image.  
            - Parts of #out_img which have no corresponding locations in in_img are set to black.
            - uses the supplied interpolation routine interp to perform the necessary
              pixel interpolation.
            - returns a transformation object that maps points in in_img into their corresponding 
              location in #out_img.
    !*/

// ----------------------------------------------------------------------------------------


    template <
        typename image_type1,
        typename image_type2
        >
    point_transform_affine rotate_image (
        const image_type1& in_img,
        image_type2& out_img,
        double angle
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type1::type>::has_alpha == false
            - pixel_traits<typename image_type2::type> is defined
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img == a copy of in_img which has been rotated angle radians counter clockwise.
              The rotation is performed with respect to the center of the image.  
            - Parts of #out_img which have no corresponding locations in in_img are set to black.
            - uses the interpolate_quadratic object to perform the necessary pixel interpolation.
            - returns a transformation object that maps points in in_img into their corresponding 
              location in #out_img.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename interpolation_type
        >
    void resize_image (
        const image_type1& in_img,
        image_type2& out_img,
        const interpolation_type& interp
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img == A copy of in_img which has been stretched so that it 
              fits exactly into out_img.   
            - The size of out_img is not modified.  I.e. 
                - #out_img.nr() == out_img.nr()
                - #out_img.nc() == out_img.nc()
            - uses the supplied interpolation routine interp to perform the necessary
              pixel interpolation.
    !*/

// ----------------------------------------------------------------------------------------


    template <
        typename image_type1,
        typename image_type2
        >
    void resize_image (
        const image_type1& in_img,
        image_type2& out_img
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type1::type>::has_alpha == false
            - pixel_traits<typename image_type2::type> is defined
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img == A copy of in_img which has been stretched so that it 
              fits exactly into out_img.   
            - The size of out_img is not modified.  I.e. 
                - #out_img.nr() == out_img.nr()
                - #out_img.nc() == out_img.nc()
            - Uses the bilinear interpolation to perform the necessary pixel interpolation.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2
        >
    point_transform_affine flip_image_left_right (
        const image_type1& in_img,
        image_type2& out_img
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type1::type> is defined
            - pixel_traits<typename image_type2::type> is defined
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img.nr() == in_img.nr()
            - #out_img.nc() == in_img.nc()
            - #out_img == a copy of in_img which has been flipped from left to right.  
              (i.e. it is flipped as if viewed though a mirror)
            - returns a transformation object that maps points in in_img into their
              corresponding location in #out_img.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type
        >
    void add_image_left_right_flips (
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
        ensures
            - This function computes all the left/right flips of the contents of images and
              then appends them onto the end of the images array.  It also finds the
              left/right flips of the rectangles in objects and similarly appends them into
              objects.  That is, we assume objects[i] is the set of bounding boxes in
              images[i] and we flip the bounding boxes so that they still bound the same
              objects in the new flipped images.
            - #images.size() == images.size()*2
            - #objects.size() == objects.size()*2
            - All the original elements of images and objects are left unmodified.  That
              is, this function only appends new elements to each of these containers.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type
        >
    void add_image_left_right_flips (
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects,
        std::vector<std::vector<rectangle> >& objects2
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
            - images.size() == objects2.size()
        ensures
            - This function computes all the left/right flips of the contents of images and
              then appends them onto the end of the images array.  It also finds the
              left/right flips of the rectangles in objects and objects2 and similarly
              appends them into objects and objects2 respectively.  That is, we assume
              objects[i] is the set of bounding boxes in images[i] and we flip the bounding
              boxes so that they still bound the same objects in the new flipped images.
              We similarly flip the boxes in objects2.
            - #images.size()   == images.size()*2
            - #objects.size()  == objects.size()*2
            - #objects2.size() == objects2.size()*2
            - All the original elements of images, objects, and objects2 are left unmodified.
              That is, this function only appends new elements to each of these containers.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type
        >
    void flip_image_dataset_left_right (
        dlib::array<image_type>& images, 
        std::vector<std::vector<rectangle> >& objects
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
        ensures
            - This function replaces each image in images with the left/right flipped
              version of the image.  Therefore, #images[i] will contain the left/right
              flipped version of images[i].  It also flips all the rectangles in objects so
              that they still bound the same visual objects in each image.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type
        >
    void flip_image_dataset_left_right (
        dlib::array<image_type>& images, 
        std::vector<std::vector<rectangle> >& objects,
        std::vector<std::vector<rectangle> >& objects2
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
            - images.size() == objects2.size()
        ensures
            - This function replaces each image in images with the left/right flipped
              version of the image.  Therefore, #images[i] will contain the left/right
              flipped version of images[i].  It also flips all the rectangles in objects
              and objects2 so that they still bound the same visual objects in each image.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - #objects2.size() == objects2.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
            - for all valid i:
                #objects2[i].size() == objects2[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename pyramid_type,
        typename image_type
        >
    void upsample_image_dataset (
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
        ensures
            - This function replaces each image in images with an upsampled version of that
              image.  Each image is upsampled using pyramid_up() and the given
              pyramid_type.  Therefore, #images[i] will contain the larger upsampled
              version of images[i].  It also adjusts all the rectangles in objects so that
              they still bound the same visual objects in each image.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename pyramid_type,
        typename image_type
        >
    void upsample_image_dataset (
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects,
        std::vector<std::vector<rectangle> >& objects2 
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
            - images.size() == objects2.size()
        ensures
            - This function replaces each image in images with an upsampled version of that
              image.  Each image is upsampled using pyramid_up() and the given
              pyramid_type.  Therefore, #images[i] will contain the larger upsampled
              version of images[i].  It also adjusts all the rectangles in objects and
              objects2 so that they still bound the same visual objects in each image.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - #objects2.size() == objects2.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
            - for all valid i:
                #objects2[i].size() == objects2[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <typename image_type>
    void rotate_image_dataset (
        double angle,
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
        ensures
            - This function replaces each image in images with a rotated version of that
              image.  In particular, each image is rotated using
              rotate_image(original,rotated,angle).  Therefore, the images are rotated
              angle radians counter clockwise around their centers. That is, #images[i]
              will contain the rotated version of images[i].  It also adjusts all
              the rectangles in objects so that they still bound the same visual objects in
              each image.
            - All the rectangles will still have the same sizes and aspect ratios after
              rotation.  They will simply have had their positions adjusted so they still
              fall on the same objects.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <typename image_type>
    void rotate_image_dataset (
        double angle,
        dlib::array<image_type>& images,
        std::vector<std::vector<rectangle> >& objects,
        std::vector<std::vector<rectangle> >& objects2
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type::type> is defined
            - images.size() == objects.size()
            - images.size() == objects2.size()
        ensures
            - This function replaces each image in images with a rotated version of that
              image.  In particular, each image is rotated using
              rotate_image(original,rotated,angle).  Therefore, the images are rotated
              angle radians counter clockwise around their centers. That is, #images[i]
              will contain the rotated version of images[i].  It also adjusts all
              the rectangles in objects and objects2 so that they still bound the same
              visual objects in each image.
            - All the rectangles will still have the same sizes and aspect ratios after
              rotation.  They will simply have had their positions adjusted so they still
              fall on the same objects.
            - #images.size() == image.size()
            - #objects.size() == objects.size()
            - #objects2.size() == objects2.size()
            - for all valid i:
                #objects[i].size() == objects[i].size()
            - for all valid i:
                #objects2[i].size() == objects2[i].size()
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2
        >
    void flip_image_up_down (
        const image_type1& in_img,
        image_type2& out_img
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type1::type> is defined
            - pixel_traits<typename image_type2::type> is defined
            - is_same_object(in_img, out_img) == false
        ensures
            - #out_img.nr() == in_img.nr()
            - #out_img.nc() == in_img.nc()
            - #out_img == a copy of in_img which has been flipped upside down.  
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename pyramid_type,
        typename interpolation_type
        >
    void pyramid_up (
        const image_type1& in_img,
        image_type2& out_img,
        const pyramid_type& pyr,
        const interpolation_type& interp
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pyramid_type == a type compatible with the image pyramid objects defined 
              in dlib/image_transforms/image_pyramid_abstract.h
            - interpolation_type == interpolate_nearest_neighbor, interpolate_bilinear, 
              interpolate_quadratic, or a type with a compatible interface.
            - is_same_object(in_img, out_img) == false
        ensures
            - This function inverts the downsampling transformation performed by pyr().
              In particular, it attempts to make an image, out_img, which would result
              in in_img when downsampled with pyr().  
            - #out_img == An upsampled copy of in_img.  In particular, downsampling
              #out_img 1 time with pyr() should result in a final image which looks like
              in_img.
            - Uses the supplied interpolation routine interp to perform the necessary
              pixel interpolation.
            - Note that downsampling an image with pyr() and then upsampling it with 
              pyramid_up() will not necessarily result in a final image which is
              the same size as the original.  This is because the exact size of the
              original image cannot be determined based on the downsampled image.
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2,
        typename pyramid_type
        >
    void pyramid_up (
        const image_type1& in_img,
        image_type2& out_img,
        const pyramid_type& pyr
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pyramid_type == a type compatible with the image pyramid objects defined 
              in dlib/image_transforms/image_pyramid_abstract.h
            - is_same_object(in_img, out_img) == false
        ensures
            - performs: pyramid_up(in_img, out_img, pyr, interpolate_bilinear());
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type,
        typename pyramid_type
        >
    void pyramid_up (
        image_type& img,
        const pyramid_type& pyr
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
            - pyramid_type == a type compatible with the image pyramid objects defined 
              in dlib/image_transforms/image_pyramid_abstract.h
        ensures
            - Performs an in-place version of pyramid_up() on the given image.  In
              particular, this function is equivalent to:
                pyramid_up(img, temp, pyr); 
                temp.swap(img);
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type
        >
    void pyramid_up (
        image_type& img
    );
    /*!
        requires
            - image_type == is an implementation of array2d/array2d_kernel_abstract.h
        ensures
            - performs: pyramid_up(img, pyramid_down<2>());
              (i.e. it upsamples the given image and doubles it in size.)
    !*/

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

    struct chip_details
    {
        /*!
            WHAT THIS OBJECT REPRESENTS
                This object describes where an image chip is to be extracted from within
                another image.  In particular, it specifies that the image chip is
                contained within the rectangle this->rect and that prior to extraction the
                image should be rotated counter-clockwise by this->angle radians.  Finally,
                the extracted chip should have this->size pixels in it regardless of the
                size of this->rect.

        !*/

        chip_details(
        ); 
        /*!
            ensures
                - #rect.is_empty() == true
                - #size == 0
                - #angle == 0
        !*/

        chip_details(
            const rectangle& rect_, 
            unsigned long size_
        );
        /*!
            ensures
                - #rect == rect_
                - #size == size_
                - #angle == 0
        !*/

        chip_details(
            const rectangle& rect_, 
            unsigned long size_,
            double angle_
        );
        /*!
            ensures
                - #rect == rect_
                - #size == size_
                - #angle == angle_
        !*/

        rectangle rect;
        unsigned long size;
        double angle;
    };

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2
        >
    void extract_image_chips (
        const image_type1& img,
        const std::vector<chip_details>& chip_locations,
        dlib::array<image_type2>& chips
    );
    /*!
        requires
            - image_type1 == is an implementation of array2d/array2d_kernel_abstract.h
            - image_type2 == is an implementation of array2d/array2d_kernel_abstract.h
            - pixel_traits<typename image_type1::type>::has_alpha == false
            - pixel_traits<typename image_type2::type> is defined
            - for all valid i: 
                - chip_locations[i].rect.is_empty() == false
                - chip_locations[i].size != 0
        ensures
            - This function extracts "chips" from an image.  That is, it takes a list of
              rectangular sub-windows (i.e. chips) within an image and extracts those
              sub-windows, storing each into its own image.  It also scales and rotates the
              image chips according to the instructions inside each chip_details object.
            - #chips == the extracted image chips
            - #chips.size() == chip_locations.size()
            - for all valid i:
                - #chips[i] == The image chip extracted from the position
                  chip_locations[i].rect in img.
                - #chips[i].nr()/#chips[i].nc() is approximately equal to
                  chip_locations[i].rect.height()/chip_locations[i].rect.width() (i.e. the
                  aspect ratio of the chip is as similar as possible to the aspect ratio of
                  the rectangle that defines the chip's location in the original image)
                - #chips[i].size() is as close to chip_locations[i].size as possible given that 
                  we attempt to keep the chip's aspect ratio similar to chip_locations[i].rect. 
                - The image will have been rotated counter-clockwise by
                  chip_locations[i].angle radians, around the center of
                  chip_locations[i].rect, before the chip was extracted. 
                - As long as chip_locations[i].size and the aspect ratio of of
                  chip_locations[i].rect stays constant then the dimensions of #chips[i] is
                  always the same.  This means that, for example, if you want all your
                  chips to have the same dimensions then ensure that chip_location[i].size
                  is always the same and also that chip_location[i].rect always has the
                  same aspect ratio.
            - Any pixels in an image chip that go outside img are set to 0 (i.e. black).
    !*/

// ----------------------------------------------------------------------------------------

    template <
        typename image_type1,
        typename image_type2
        >
    void extract_image_chip (
        const image_type1& img,
        const chip_details& chip_location,
        image_type2& chip
    );
    /*!
        ensures
            - This function simply calls extract_image_chips() with a single chip location
              and stores the single output chip into #chip.
    !*/

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

}

#endif // DLIB_INTERPOlATION_ABSTRACT_

