"""
An partial alternative to cv_bridge that does not depends on opencv
"""

import sensor_msgs.msg


class Bridge:

    numpy_type_to_cvtype = {
        'uint8': '8U',
        'int8': '8S',
        'uint16': '16U',
        'int16': '16S',
        'int32': '32S',
        'float32': '32F',
        'float64': '64F'
    }

    def cv2_to_imgmsg(self, cvim, encoding='passthrough', header=None):
        """
        Same as original CvBridge but does not check the
        encoding correctness
        """
        import numpy as np
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = sensor_msgs.msg.Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if header is not None:
            img_msg.header = header
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(
                cvim.dtype, cvim.shape[2])
        if encoding == 'passthrough':
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data.frombytes(cvim.tobytes())
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return f'{self.numpy_type_to_cvtype[dtype.name]}C{n_channels}'
