static unsigned char ov9655_sxga[] = {
0x12, 0x80,
0x00, 0x00,
0x01, 0x80,
0x02, 0x80,
0x03, 0x1b,
0x04, 0x03,
0x0b, 0x57,
0x0e, 0x61,
0x0f, 0x40,
0x11, 0x01,
0x12, 0x02,
0x13, 0xc7,  // was e7 - turned banding filter off
0x14, 0x3a,
0x16, 0x24,
0x17, 0x1d,
0x18, 0xbd,
0x19, 0x01,
0x1a, 0x81,
0x1e, 0x04,
0x24, 0x3c,
0x25, 0x36,
0x26, 0x72,
0x27, 0x08,
0x28, 0x08,
0x29, 0x15,
0x2a, 0x00,
0x2b, 0x00,
0x2c, 0x08,
0x32, 0xff,
0x33, 0x00,
0x34, 0x3d,
0x35, 0x00,
0x36, 0xf8,
0x38, 0x72,
0x39, 0x57,
0x3a, 0x8c,  // UYVY capture
0x3b, 0x04,
0x3d, 0x99,
0x3e, 0x0c,
0x3f, 0xc1,
0x40, 0xc0,
0x41, 0x00,
0x42, 0xc0,
0x43, 0x0a,
0x44, 0xf0,
0x45, 0x46,
0x46, 0x62,
0x47, 0x2a,
0x48, 0x3c,
0x4a, 0xfc,
0x4b, 0xfc,
0x4c, 0x7f,
0x4d, 0x7f,
0x4e, 0x7f,
0x4f, 0x98,
0x50, 0x98,
0x51, 0x00,
0x52, 0x28,
0x53, 0x88,
0x54, 0xb0,
0x58, 0x1a,
0x58, 0x1a,
0x59, 0x85,
0x5a, 0xa9,
0x5b, 0x64,
0x5c, 0x84,
0x5d, 0x53,
0x5e, 0x0e,
0x5f, 0xf0,
0x60, 0xf0,
0x61, 0xf0,
0x62, 0x00,
0x63, 0x00,
0x64, 0x02,
0x65, 0x16,
0x66, 0x01,
0x69, 0x02,
0x6b, 0x5a,
0x6c, 0x04,
0x6d, 0x55,
0x6e, 0x00,
0x6f, 0x9d,
0x70, 0x21,
0x71, 0x78,
0x72, 0x00,
0x73, 0x01,
0x74, 0x3a,
0x75, 0x35,
0x76, 0x01,
0x77, 0x02,
0x7a, 0x12,
0x7b, 0x8,
0x7c, 0x15,
0x7d, 0x24,
0x7e, 0x45,
0x7f, 0x55,
0x80, 0x6a,
0x81, 0x78,
0x82, 0x87,
0x83, 0x96,
0x84, 0xa3,
0x85, 0xb4,
0x86, 0xc3,
0x87, 0xd6,
0x88, 0xe6,
0x89, 0xf2,
0x8a, 0x03,
0x8c, 0x0d,
0x90, 0x7d,
0x91, 0x7b,
0x9d, 0x03,
0x9e, 0x04,
0x9f, 0x7a,
0xa0, 0x79,
0xa1, 0x10,   //  changes exposure time - default was 0x40
0xa4, 0x50,
0xa5, 0x68,
0xa6, 0x4a,
0xa8, 0xc1,
0xa9, 0xef,
0xaa, 0x92,
0xab, 0x04,
0xac, 0x80,
0xad, 0x80,
0xae, 0x80,
0xaf, 0x80,
0xb2, 0xf2,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x00,
0xb6, 0xaf,
0xbb, 0xae,
0xbc, 0x7f,
0xbd, 0x7f,
0xbe, 0x7f,
0xbf, 0x7f,
0xc0, 0xe2,
0xc1, 0xc0,
0xc2, 0x01,
0xc3, 0x4e,
0xc6, 0x05,
0xc7, 0x80,
0xc9, 0xe0,
0xca, 0xe8,
0xcb, 0xf0,
0xcc, 0xd8,
0xcd, 0x93,
};

static unsigned char ov9655_vga[] = {
0x12, 0x80,
0x00, 0x00,
0x01, 0x80,
0x02, 0x80,
0x03, 0x12,
0x04, 0x03,
0x0b, 0x57,
0x0e, 0x61,
0x0f, 0x40,
0x11, 0x01,
0x12, 0x62,
0x13, 0xc7,
0x14, 0x3a,
0x16, 0x24,
0x17, 0x16,
0x18, 0x02,
0x19, 0x01,
0x1a, 0x3d,
0x1e, 0x04,
0x24, 0x3c,
0x25, 0x36,
0x26, 0x72,
0x27, 0x08,
0x28, 0x08,
0x29, 0x15,
0x2a, 0x00,
0x2b, 0x00,
0x2c, 0x08,
0x32, 0xff,
0x33, 0x00,
0x34, 0x3F,
0x35, 0x00,
0x36, 0xfa,
0x38, 0x72,
0x39, 0x57,
0x3a, 0x8c,
0x3b, 0x04,
0x3d, 0x99,
0x3e, 0x0c,
0x3f, 0xc1,
0x40, 0xc0,
0x41, 0x00,
0x42, 0xc0,
0x43, 0x0a,
0x44, 0xf0,
0x45, 0x46,
0x46, 0x62,
0x47, 0x2a,
0x48, 0x3c,
0x4a, 0xfc,
0x4b, 0xfc,
0x4c, 0x7f,
0x4d, 0x7f,
0x4e, 0x7f,
0x4f, 0x98,
0x50, 0x98,
0x51, 0x00,
0x52, 0x28,
0x53, 0x70,
0x54, 0x98,
0x58, 0x1a,
0x59, 0x85,
0x5a, 0xa9,
0x5b, 0x64,
0x5c, 0x84,
0x5d, 0x53,
0x5e, 0x0e,
0x5f, 0xf0,
0x60, 0xf0,
0x61, 0xf0,
0x62, 0x00,
0x63, 0x00,
0x64, 0x02,
0x65, 0x20,
0x66, 0x00,
0x69, 0x0a,
0x6b, 0x5a,
0x6c, 0x04,
0x6d, 0x55,
0x6e, 0x00,
0x6f, 0x9d,
0x70, 0x21,
0x71, 0x78,
0x72, 0x00,
0x73, 0x00,
0x74, 0x3a,
0x75, 0x35,
0x76, 0x01,
0x77, 0x02,
0x7A, 0x12,
0x7B, 0x08,
0x7C, 0x16,
0x7D, 0x30,
0x7E, 0x5e,
0x7F, 0x72,
0x80, 0x82,
0x81, 0x8e,
0x82, 0x9a,
0x83, 0xa4,
0x84, 0xac,
0x85, 0xb8,
0x86, 0xc3,
0x87, 0xd6,
0x88, 0xe6,
0x89, 0xf2,
0x8a, 0x24,
0x8c, 0x8d,
0x90, 0x7d,
0x91, 0x7b,
0x9d, 0x02,
0x9e, 0x02,
0x9f, 0x7a,
0xa0, 0x79,
0xa1, 0x40,
0xa4, 0x50,
0xa5, 0x68,
0xa6, 0x4a,
0xa8, 0xc1,
0xa9, 0xef,
0xaa, 0x92,
0xab, 0x04,
0xac, 0x80,
0xad, 0x80,
0xae, 0x80,
0xaf, 0x80,
0xb2, 0xf2,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x00,
0xb6, 0xaf,
0xbb, 0xae,
0xbc, 0x7f,
0xbd, 0x7f,
0xbe, 0x7f,
0xbf, 0x7f,
0xc0, 0xaa,
0xc1, 0xc0,
0xc2, 0x01,
0xc3, 0x4e,
0xc6, 0x05,
0xc7, 0x80,
0xc9, 0xe0,
0xca, 0xe8,
0xcb, 0xf0,
0xcc, 0xd8,
0xcd, 0x93,
0xcd, 0x93,
};

static unsigned char ov9655_qvga[] = {
0x12, 0x80,
0x00, 0x00,
0x01, 0x80,
0x02, 0x80,
0x03, 0x02,
0x04, 0x03,
0x0b, 0x57,
0x0e, 0x61,
0x0f, 0x40,
0x11, 0x01,
0x12, 0x62,
0x13, 0xc7,
0x14, 0x3a,
0x16, 0x24,
0x17, 0x18,
0x18, 0x04,
0x19, 0x01,
0x1a, 0x81,
0x1e, 0x04,
0x24, 0x3c,
0x25, 0x36,
0x26, 0x72,
0x27, 0x08,
0x28, 0x08,
0x29, 0x15,
0x2a, 0x00,
0x2b, 0x00,
0x2c, 0x08,
0x32, 0x24,
0x33, 0x00,
0x34, 0x3f,
0x35, 0x00,
0x36, 0x3a,
0x38, 0x72,
0x39, 0x57,
0x3a, 0x8c,
0x3b, 0x04,
0x3d, 0x99,
0x3e, 0x0e,
0x3f, 0xc1,
0x40, 0xc0,
0x41, 0x01,
0x42, 0xc0,
0x43, 0x0a,
0x44, 0xf0,
0x45, 0x46,
0x46, 0x62,
0x47, 0x2a,
0x48, 0x3c,
0x4a, 0xfc,
0x4b, 0xfc,
0x4c, 0x7f,
0x4d, 0x7f,
0x4e, 0x7f,
0x4f, 0x98,
0x50, 0x98,
0x51, 0x00,
0x52, 0x28,
0x53, 0x70,
0x54, 0x98,
0x58, 0x1a,
0x59, 0x85,
0x5a, 0xa9,
0x5b, 0x64,
0x5c, 0x84,
0x5d, 0x53,
0x5e, 0x0e,
0x5f, 0xf0,
0x60, 0xf0,
0x61, 0xf0,
0x62, 0x00,
0x63, 0x00,
0x64, 0x02,
0x65, 0x20,
0x66, 0x00,
0x69, 0x0a,
0x6b, 0x5a,
0x6c, 0x04,
0x6d, 0x55,
0x6e, 0x00,
0x6f, 0x9d,
0x70, 0x21,
0x71, 0x78,
0x72, 0x11,
0x73, 0x01,
0x74, 0x10,
0x75, 0x10,
0x76, 0x01,
0x77, 0x02,
0x7A, 0x12,
0x7B, 0x08,
0x7C, 0x16,
0x7D, 0x30,
0x7E, 0x5e,
0x7F, 0x72,
0x80, 0x82,
0x81, 0x8e,
0x82, 0x9a,
0x83, 0xa4,
0x84, 0xac,
0x85, 0xb8,
0x86, 0xc3,
0x87, 0xd6,
0x88, 0xe6,
0x89, 0xf2,
0x8a, 0x24,
0x8c, 0x80,
0x90, 0x7d,
0x91, 0x7b,
0x9d, 0x02,
0x9e, 0x02,
0x9f, 0x7a,
0xa0, 0x79,
0xa1, 0x40,
0xa4, 0x50,
0xa5, 0x68,
0xa6, 0x4a,
0xa8, 0xc1,
0xa9, 0xef,
0xaa, 0x92,
0xab, 0x04,
0xac, 0x80,
0xad, 0x80,
0xae, 0x80,
0xaf, 0x80,
0xb2, 0xf2,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x00,
0xb6, 0xaf,
0xbb, 0xae,
0xbc, 0x7f,
0xbd, 0x7f,
0xbe, 0x7f,
0xbf, 0x7f,
0xc0, 0xaa,
0xc1, 0xc0,
0xc2, 0x01,
0xc3, 0x4e,
0xc6, 0x05,
0xc7, 0x81,
0xc9, 0xe0,
0xca, 0xe8,
0xcb, 0xf0,
0xcc, 0xd8,
0xcd, 0x93,
};


static unsigned char ov9655_qqvga[] = {
0x12, 0x80,
0x00, 0x00,
0x01, 0x80,
0x02, 0x80,
0x03, 0x02,
0x04, 0x03,
0x0b, 0x57,
0x0e, 0x61,
0x0f, 0x40,
0x11, 0x01,
0x12, 0x62,
0x13, 0xc7,
0x14, 0x3a,
0x16, 0x24,
0x17, 0x18,
0x18, 0x04,
0x19, 0x01,
0x1a, 0x81,
0x1e, 0x04,
0x24, 0x3c,
0x25, 0x36,
0x26, 0x72,
0x27, 0x08,
0x28, 0x08,
0x29, 0x15,
0x2a, 0x00,
0x2b, 0x00,
0x2c, 0x08,
0x32, 0xa4,
0x33, 0x00,
0x34, 0x3f,
0x35, 0x00,
0x36, 0x3a,
0x38, 0x72,
0x39, 0x57,
0x3a, 0x8c,
0x3b, 0x04,
0x3d, 0x99,
0x3e, 0x0e,
0x3f, 0xc1,
0x40, 0xc0,
0x41, 0x01,
0x42, 0xc0,
0x43, 0x0a,
0x44, 0xf0,
0x45, 0x46,
0x46, 0x62,
0x47, 0x2a,
0x48, 0x3c,
0x4a, 0xfc,
0x4b, 0xfc,
0x4c, 0x7f,
0x4d, 0x7f,
0x4e, 0x7f,
0x4f, 0x98,
0x50, 0x98,
0x51, 0x00,
0x52, 0x28,
0x53, 0x70,
0x54, 0x98,
0x58, 0x1a,
0x59, 0x85,
0x5a, 0xa9,
0x5b, 0x64,
0x5c, 0x84,
0x5d, 0x53,
0x5e, 0x0e,
0x5f, 0xf0,
0x60, 0xf0,
0x61, 0xf0,
0x62, 0x00,
0x63, 0x00,
0x64, 0x02,
0x65, 0x20,
0x66, 0x00,
0x69, 0x0a,
0x6b, 0x5a,
0x6c, 0x04,
0x6d, 0x55,
0x6e, 0x00,
0x6f, 0x9d,
0x70, 0x21,
0x71, 0x78,
0x72, 0x22,
0x73, 0x02,
0x74, 0x10,
0x75, 0x10,
0x76, 0x01,
0x77, 0x02,
0x7A, 0x12,
0x7B, 0x08,
0x7C, 0x16,
0x7D, 0x30,
0x7E, 0x5e,
0x7F, 0x72,
0x80, 0x82,
0x81, 0x8e,
0x82, 0x9a,
0x83, 0xa4,
0x84, 0xac,
0x85, 0xb8,
0x86, 0xc3,
0x87, 0xd6,
0x88, 0xe6,
0x89, 0xf2,
0x8a, 0x24,
0x8c, 0x80,
0x90, 0x7d,
0x91, 0x7b,
0x9d, 0x02,
0x9e, 0x02,
0x9f, 0x7a,
0xa0, 0x79,
0xa1, 0x40,
0xa4, 0x50,
0xa5, 0x68,
0xa6, 0x4a,
0xa8, 0xc1,
0xa9, 0xef,
0xaa, 0x92,
0xab, 0x04,
0xac, 0x80,
0xad, 0x80,
0xae, 0x80,
0xaf, 0x80,
0xb2, 0xf2,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x00,
0xb6, 0xaf,
0xbb, 0xae,
0xbc, 0x7f,
0xbd, 0x7f,
0xbe, 0x7f,
0xbf, 0x7f,
0xc0, 0xaa,
0xc1, 0xc0,
0xc2, 0x01,
0xc3, 0x4e,
0xc6, 0x05,
0xc7, 0x82,
0xc9, 0xe0,
0xca, 0xe8,
0xcb, 0xf0,
0xcc, 0xd8,
0xcd, 0x93,
};

static unsigned char ov9655_invert[] = {
0x1E, 0x30,
};

static unsigned char ov9655_restore[] = {
0x1E, 0x00,
};

