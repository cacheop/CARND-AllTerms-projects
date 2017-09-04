def closest(list, number):
    """
    Calculate the two closest elements in value 
    to given input number in a list
    """
    aux = []
    for valor in list:
        aux.append(abs(number-valor))
    first_lower = aux.index(min(aux))
    first_higher = first_lower+1

    return first_lower, first_higher

def grid_sizes(n):
    """
    Defines a rectangular grid size for n elements. If n
    is prime it returns n x 1 🙁
    """
    print(n)
    fn = []
    sq = int(np.sqrt(n))

    for x in range(2,n):
        if n%x == 0:
            fn.append(x)
    if (len(fn)>0):
        c, r = closest(fn, sq)
        col = fn[c]
        row = fn[r]
    else:
        col = 1
        row = n
        
    return col, row
        
def images_square_grid(images, mode):
    """
    Save images as a square grid
    :param images: Images to be used for the grid
    :param mode: The mode to use for images
    :return: Image of images in a square grid
    """
    # Get maximum size for square grid of images
    save_size = math.floor(np.sqrt(images.shape[0]))

    # Scale to 0-255
    images = (((images - images.min()) * 255) / (images.max() - images.min())).astype(np.uint8)

    # Put images in a square arrangement
    images_in_square = np.reshape(
            images[:save_size*save_size],
            (save_size, save_size, images.shape[1], images.shape[2], images.shape[3]))
    if mode == 'L':
        images_in_square = np.squeeze(images_in_square, 4)

    # Combine images to grid image
    new_im = Image.new(mode, (images.shape[1] * save_size, images.shape[2] * save_size))
    for col_i, col_images in enumerate(images_in_square):
        for image_i, image in enumerate(col_images):
            im = Image.fromarray(image, mode)
            new_im.paste(im, (col_i * images.shape[1], image_i * images.shape[2]))

    return new_im

def get_batch(image_files, width, height, mode):
    data_batch = np.array(
        [get_image(sample_file, width, height, mode) for sample_file in image_files]).astype(np.float32)

    # Make sure the images are in 4 dimensions
    if len(data_batch.shape) < 4:
        data_batch = data_batch.reshape(data_batch.shape + (1,))

    return data_batch