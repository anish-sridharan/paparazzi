/*
 * Copyright (C) 2016 G. de Croon <guido.de.croon@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/lib/vision/qr_code_recognition.c
 * @brief Efficient recognition of QR-codes. Note: only applicable to a very limited number of codes! Not generic like z-lib.
 *
 */

#include "qr_code_recognition.h"
#include "inttypes.h"
#include "integral_image.h"

// kind of cell grid to use for the integral features:
#define N_CELLS_1 3
#define N_CELLS_2 10
int cell_structure[2] = {N_CELLS_1, N_CELLS_2};
int length_cs = 2;
#define N_FEATURES (N_CELLS_1*N_CELLS_1+N_CELLS_2*N_CELLS_2)

// get a QR code from an entire image:
// watch out, the image should be (almost) filled with the QR code in order to work. 
// also, it should be straight up in the image
int get_QR_class(struct image_t *img, float* uncertainty)
{
  int class;
  float integral_features[N_FEATURES];

  // make an integral image the size of the input image:
  uint32_t integral_image[img->w*img->h];
  get_integral_image(img->buf, (uint32_t) img->w, (uint32_t) img->h, integral_image);

  // get the integral features:
  get_QR_integral_features(integral_image, (uint32_t) img->w, (uint32_t) img->h, integral_features);

  // classify the features:
  QR_J48(integral_features, &class, uncertainty);

  return class;
}


// get a QR code from a region of interest: 
// Note: this tree has been trained on roughly 70x70 pixel patches. Bigger patches are definitely no problem, for smaller it is not clear.
// The ROI should almost only contain the QR code. Classifications will always be high confidence, as we do not have a "non-QR code" class.
int get_QR_class_ROI(struct image_t *img, uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max, float* uncertainty)
{
  int class;  

  // make an integral image the size of the ROI:
  uint32_t width = x_max - x_min;
  uint32_t height = y_max - y_min;  
  uint32_t integral_image[width*height];
  get_integral_image_ROI(img->buf, (uint32_t) img->w, (uint32_t) img->h, integral_image, x_min, y_min, x_max, y_max);

  // get the integral features:
  float integral_features[N_FEATURES];
  get_QR_integral_features(integral_image, width, height, integral_features);

  // classify the features:
  QR_J48(integral_features, &class, uncertainty);

  return class;
}

// get integral features that will be used for classification:
void get_QR_integral_features(uint32_t* integral_image, uint32_t width, uint32_t height, float* features)
{
  // get the global illuminance, in order to make all features relative to that 
  float global_ill = (float) get_avg_ii(0, 0, width, height, integral_image, width, height);
  uint16_t cs, n_cells, cll, n_rows, r, c;
  uint32_t cell_size, avg, x, y;

  // loop over the cell structure:
  cll = 0;
  for(cs = 0; cs < length_cs; cs++)
  {
    // the cell structure defines the number of rows = columns
    n_rows = cell_structure[cs];
    n_cells = n_rows * n_rows;
    cell_size = height / n_rows;

    // for each cell in the "matrix" obtain the average illuminance:
    for(r = 0; r < n_rows; r++)
    {
      y = r * cell_size;
      for(c = 0; c < n_rows; c++)
      {
        x = c * cell_size;
        // get the average illuminance in the current cell:
        avg = get_avg_ii(x, y, x+cell_size, y+cell_size, integral_image, width, height);

        // divide the average illuminance by the average global illuminance to cope with global illuminance changes:
        features[cll] = ((float) avg) / global_ill;

        // increment the number of features already extracted:
        cll++;
      }
    }
  }
}

// apply a J48 tree learned with WEKA and then translated in MATLAB to C-code
// - attributes are the features of a single instance
// - class and uncertainty of the classification are returned.
void QR_J48(float* attributes, int* class, float* uncertainty)
{
	// use a WEKA-learned tree to classify an instance as defined by its attributes:
	// Returns an integer class and an uncertainty in the range [0,1].
	if(attributes[4] <= 0.101402)
	{
		if(attributes[52] <= 0.007037)
		{
			if(attributes[62] <= 0.006590)
			{
				if(attributes[63] <= 0.006735)
				{
					if(attributes[64] <= 0.006657)
					{
						if(attributes[106] <= 0.008828)
						{
							// QR_2: 100%
							(*uncertainty) = 0.000000;
							(*class) = 2;
							return;
						}
						else
						{
							// QR_25: 100%
							(*uncertainty) = 0.000000;
							(*class) = 25;
							return;
						}
					}
					else
					{
						if(attributes[49] <= 0.007254)
						{
							// QR_1: 98%
							(*uncertainty) = 0.027027;
							(*class) = 1;
							return;
						}
						else
						{
							// QR_2: 99%
							(*uncertainty) = 0.019608;
							(*class) = 2;
							return;
						}
					}
				}
				else
				{
					if(attributes[66] <= 0.009477)
					{
						if(attributes[65] <= 0.008033)
						{
							if(attributes[86] <= 0.006598)
							{
								if(attributes[50] <= 0.006449)
								{
									// QR_1: 80%
									(*uncertainty) = 0.200000;
									(*class) = 1;
									return;
								}
								else
								{
									// QR_5: 100%
									(*uncertainty) = 0.000000;
									(*class) = 5;
									return;
								}
							}
							else
							{
								if(attributes[85] <= 0.009679)
								{
									if(attributes[8] <= 0.105488)
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
									else
									{
										// QR_13: 100%
										(*uncertainty) = 0.000000;
										(*class) = 13;
										return;
									}
								}
								else
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
							}
						}
						else
						{
							// QR_4: 100%
							(*uncertainty) = 0.000000;
							(*class) = 4;
							return;
						}
					}
					else
					{
						// QR_2: 100%
						(*uncertainty) = 0.009709;
						(*class) = 2;
						return;
					}
				}
			}
			else
			{
				if(attributes[54] <= 0.007252)
				{
					if(attributes[43] <= 0.005554)
					{
						// QR_1: 100%
						(*uncertainty) = 0.000000;
						(*class) = 1;
						return;
					}
					else
					{
						// QR_14: 100%
						(*uncertainty) = 0.000000;
						(*class) = 14;
						return;
					}
				}
				else
				{
					if(attributes[44] <= 0.006650)
					{
						// QR_1: 100%
						(*uncertainty) = 0.002086;
						(*class) = 1;
						return;
					}
					else
					{
						if(attributes[61] <= 0.007705)
						{
							if(attributes[66] <= 0.008063)
							{
								if(attributes[76] <= 0.005422)
								{
									// QR_7: 100%
									(*uncertainty) = 0.000000;
									(*class) = 7;
									return;
								}
								else
								{
									// QR_4: 100%
									(*uncertainty) = 0.003630;
									(*class) = 4;
									return;
								}
							}
							else
							{
								if(attributes[88] <= 0.007190)
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
								else
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
							}
						}
						else
						{
							if(attributes[51] <= 0.007593)
							{
								// QR_1: 94%
								(*uncertainty) = 0.062500;
								(*class) = 1;
								return;
							}
							else
							{
								if(attributes[106] <= 0.007236)
								{
									// QR_2: 100%
									(*uncertainty) = 0.000000;
									(*class) = 2;
									return;
								}
								else
								{
									// QR_25: 100%
									(*uncertainty) = 0.000000;
									(*class) = 25;
									return;
								}
							}
						}
					}
				}
			}
		}
		else
		{
			if(attributes[43] <= 0.008027)
			{
				if(attributes[23] <= 0.008410)
				{
					if(attributes[63] <= 0.008543)
					{
						if(attributes[66] <= 0.006526)
						{
							if(attributes[65] <= 0.006237)
							{
								if(attributes[52] <= 0.007969)
								{
									if(attributes[0] <= 0.090521)
									{
										// QR_9: 100%
										(*uncertainty) = 0.000000;
										(*class) = 9;
										return;
									}
									else
									{
										// QR_7: 100%
										(*uncertainty) = 0.000000;
										(*class) = 7;
										return;
									}
								}
								else
								{
									// QR_9: 100%
									(*uncertainty) = 0.000000;
									(*class) = 9;
									return;
								}
							}
							else
							{
								if(attributes[75] <= 0.008017)
								{
									if(attributes[43] <= 0.006525)
									{
										// QR_1: 80%
										(*uncertainty) = 0.200000;
										(*class) = 1;
										return;
									}
									else
									{
										// QR_2: 100%
										(*uncertainty) = 0.000000;
										(*class) = 2;
										return;
									}
								}
								else
								{
									// QR_4: 100%
									(*uncertainty) = 0.000000;
									(*class) = 4;
									return;
								}
							}
						}
						else
						{
							if(attributes[51] <= 0.006850)
							{
								if(attributes[66] <= 0.007966)
								{
									if(attributes[35] <= 0.008533)
									{
										if(attributes[12] <= 0.007814)
										{
											// QR_2: 75%
											(*uncertainty) = 0.250000;
											(*class) = 2;
											return;
										}
										else
										{
											// QR_7: 67%
											(*uncertainty) = 0.333333;
											(*class) = 7;
											return;
										}
									}
									else
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
								}
								else
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
							}
							else
							{
								if(attributes[43] <= 0.006087)
								{
									if(attributes[45] <= 0.008006)
									{
										if(attributes[1] <= 0.107363)
										{
											// QR_1: 67%
											(*uncertainty) = 0.333333;
											(*class) = 1;
											return;
										}
										else
										{
											// QR_1: 100%
											(*uncertainty) = 0.000000;
											(*class) = 1;
											return;
										}
									}
									else
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
								}
								else
								{
									if(attributes[44] <= 0.008600)
									{
										if(attributes[76] <= 0.006920)
										{
											// QR_3: 94%
											(*uncertainty) = 0.066667;
											(*class) = 3;
											return;
										}
										else
										{
											if(attributes[30] <= 0.007105)
											{
												// QR_2: 100%
												(*uncertainty) = 0.000000;
												(*class) = 2;
												return;
											}
											else
											{
												// QR_23: 100%
												(*uncertainty) = 0.000000;
												(*class) = 23;
												return;
											}
										}
									}
									else
									{
										// QR_2: 100%
										(*uncertainty) = 0.000000;
										(*class) = 2;
										return;
									}
								}
							}
						}
					}
					else
					{
						if(attributes[51] <= 0.006447)
						{
							if(attributes[74] <= 0.005488)
							{
								if(attributes[73] <= 0.008425)
								{
									if(attributes[40] <= 0.007330)
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
									else
									{
										// QR_11: 100%
										(*uncertainty) = 0.000000;
										(*class) = 11;
										return;
									}
								}
								else
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
							}
							else
							{
								// QR_4: 100%
								(*uncertainty) = 0.000000;
								(*class) = 4;
								return;
							}
						}
						else
						{
							// QR_1: 100%
							(*uncertainty) = 0.001067;
							(*class) = 1;
							return;
						}
					}
				}
				else
				{
					if(attributes[54] <= 0.004948)
					{
						if(attributes[50] <= 0.007281)
						{
							// QR_3: 100%
							(*uncertainty) = 0.000000;
							(*class) = 3;
							return;
						}
						else
						{
							if(attributes[58] <= 0.007544)
							{
								if(attributes[82] <= 0.006173)
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
								else
								{
									// QR_18: 100%
									(*uncertainty) = 0.000000;
									(*class) = 18;
									return;
								}
							}
							else
							{
								// QR_7: 100%
								(*uncertainty) = 0.000000;
								(*class) = 7;
								return;
							}
						}
					}
					else
					{
						if(attributes[59] <= 0.005606)
						{
							if(attributes[60] <= 0.007625)
							{
								// QR_7: 100%
								(*uncertainty) = 0.000000;
								(*class) = 7;
								return;
							}
							else
							{
								if(attributes[66] <= 0.006084)
								{
									// QR_9: 67%
									(*uncertainty) = 0.333333;
									(*class) = 9;
									return;
								}
								else
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
							}
						}
						else
						{
							if(attributes[13] <= 0.006613)
							{
								if(attributes[90] <= 0.004506)
								{
									// QR_1: 100%
									(*uncertainty) = 0.000000;
									(*class) = 1;
									return;
								}
								else
								{
									// QR_18: 100%
									(*uncertainty) = 0.000000;
									(*class) = 18;
									return;
								}
							}
							else
							{
								if(attributes[73] <= 0.007426)
								{
									// QR_1: 100%
									(*uncertainty) = 0.002235;
									(*class) = 1;
									return;
								}
								else
								{
									if(attributes[57] <= 0.008092)
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
									else
									{
										// QR_17: 100%
										(*uncertainty) = 0.000000;
										(*class) = 17;
										return;
									}
								}
							}
						}
					}
				}
			}
			else
			{
				if(attributes[62] <= 0.006517)
				{
					if(attributes[53] <= 0.005974)
					{
						// QR_2: 100%
						(*uncertainty) = 0.000000;
						(*class) = 2;
						return;
					}
					else
					{
						// QR_23: 100%
						(*uncertainty) = 0.000000;
						(*class) = 23;
						return;
					}
				}
				else
				{
					if(attributes[56] <= 0.006941)
					{
						if(attributes[43] <= 0.008743)
						{
							// QR_9: 100%
							(*uncertainty) = 0.000000;
							(*class) = 9;
							return;
						}
						else
						{
							// QR_1: 100%
							(*uncertainty) = 0.000000;
							(*class) = 1;
							return;
						}
					}
					else
					{
						if(attributes[50] <= 0.008061)
						{
							if(attributes[0] <= 0.089360)
							{
								// QR_3: 100%
								(*uncertainty) = 0.000000;
								(*class) = 3;
								return;
							}
							else
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
						}
						else
						{
							// QR_1: 100%
							(*uncertainty) = 0.000000;
							(*class) = 1;
							return;
						}
					}
				}
			}
		}
	}
	else
	{
		if(attributes[52] <= 0.008771)
		{
			if(attributes[51] <= 0.008429)
			{
				if(attributes[74] <= 0.005790)
				{
					if(attributes[34] <= 0.008673)
					{
						if(attributes[53] <= 0.005741)
						{
							if(attributes[48] <= 0.006465)
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
							else
							{
								// QR_14: 100%
								(*uncertainty) = 0.000000;
								(*class) = 14;
								return;
							}
						}
						else
						{
							if(attributes[63] <= 0.008107)
							{
								if(attributes[57] <= 0.006718)
								{
									// QR_2: 100%
									(*uncertainty) = 0.000000;
									(*class) = 2;
									return;
								}
								else
								{
									// QR_21: 100%
									(*uncertainty) = 0.000000;
									(*class) = 21;
									return;
								}
							}
							else
							{
								// QR_2: 100%
								(*uncertainty) = 0.002157;
								(*class) = 2;
								return;
							}
						}
					}
					else
					{
						if(attributes[64] <= 0.009053)
						{
							if(attributes[52] <= 0.006313)
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
							else
							{
								// QR_13: 100%
								(*uncertainty) = 0.000000;
								(*class) = 13;
								return;
							}
						}
						else
						{
							if(attributes[33] <= 0.007399)
							{
								// QR_4: 100%
								(*uncertainty) = 0.000000;
								(*class) = 4;
								return;
							}
							else
							{
								if(attributes[20] <= 0.000977)
								{
									// QR_2: 100%
									(*uncertainty) = 0.000000;
									(*class) = 2;
									return;
								}
								else
								{
									// QR_20: 100%
									(*uncertainty) = 0.000000;
									(*class) = 20;
									return;
								}
							}
						}
					}
				}
				else
				{
					if(attributes[1] <= 0.102630)
					{
						if(attributes[73] <= 0.007382)
						{
							if(attributes[35] <= 0.007160)
							{
								if(attributes[61] <= 0.007291)
								{
									// QR_2: 100%
									(*uncertainty) = 0.000000;
									(*class) = 2;
									return;
								}
								else
								{
									// QR_20: 100%
									(*uncertainty) = 0.000000;
									(*class) = 20;
									return;
								}
							}
							else
							{
								// QR_2: 97%
								(*uncertainty) = 0.033333;
								(*class) = 2;
								return;
							}
						}
						else
						{
							if(attributes[34] <= 0.007131)
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
							else
							{
								// QR_15: 100%
								(*uncertainty) = 0.000000;
								(*class) = 15;
								return;
							}
						}
					}
					else
					{
						if(attributes[54] <= 0.007090)
						{
							if(attributes[61] <= 0.006883)
							{
								if(attributes[63] <= 0.008181)
								{
									// QR_2: 99%
									(*uncertainty) = 0.018182;
									(*class) = 2;
									return;
								}
								else
								{
									if(attributes[49] <= 0.009490)
									{
										if(attributes[49] <= 0.006116)
										{
											// QR_4: 100%
											(*uncertainty) = 0.000000;
											(*class) = 4;
											return;
										}
										else
										{
											// QR_2: 100%
											(*uncertainty) = 0.000000;
											(*class) = 2;
											return;
										}
									}
									else
									{
										// QR_8: 100%
										(*uncertainty) = 0.000000;
										(*class) = 8;
										return;
									}
								}
							}
							else
							{
								if(attributes[62] <= 0.006886)
								{
									// QR_1: 90%
									(*uncertainty) = 0.100000;
									(*class) = 1;
									return;
								}
								else
								{
									if(attributes[34] <= 0.007350)
									{
										// QR_1: 100%
										(*uncertainty) = 0.000000;
										(*class) = 1;
										return;
									}
									else
									{
										// QR_19: 100%
										(*uncertainty) = 0.000000;
										(*class) = 19;
										return;
									}
								}
							}
						}
						else
						{
							if(attributes[23] <= 0.006482)
							{
								if(attributes[61] <= 0.007039)
								{
									if(attributes[53] <= 0.007354)
									{
										if(attributes[44] <= 0.007586)
										{
											// QR_2: 100%
											(*uncertainty) = 0.000000;
											(*class) = 2;
											return;
										}
										else
										{
											// QR_23: 100%
											(*uncertainty) = 0.000000;
											(*class) = 23;
											return;
										}
									}
									else
									{
										if(attributes[53] <= 0.012699)
										{
											if(attributes[54] <= 0.011142)
											{
												// QR_8: 100%
												(*uncertainty) = 0.000000;
												(*class) = 8;
												return;
											}
											else
											{
												if(attributes[1] <= 0.116524)
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
												else
												{
													// QR_22: 100%
													(*uncertainty) = 0.000000;
													(*class) = 22;
													return;
												}
											}
										}
										else
										{
											// QR_2: 100%
											(*uncertainty) = 0.000000;
											(*class) = 2;
											return;
										}
									}
								}
								else
								{
									if(attributes[64] <= 0.007465)
									{
										if(attributes[66] <= 0.008118)
										{
											// QR_1: 100%
											(*uncertainty) = 0.002907;
											(*class) = 1;
											return;
										}
										else
										{
											// QR_2: 100%
											(*uncertainty) = 0.000000;
											(*class) = 2;
											return;
										}
									}
									else
									{
										if(attributes[73] <= 0.008068)
										{
											// QR_6: 100%
											(*uncertainty) = 0.000000;
											(*class) = 6;
											return;
										}
										else
										{
											if(attributes[61] <= 0.008834)
											{
												if(attributes[13] <= 0.006613)
												{
													// QR_8: 67%
													(*uncertainty) = 0.333333;
													(*class) = 8;
													return;
												}
												else
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
											}
											else
											{
												// QR_2: 100%
												(*uncertainty) = 0.000000;
												(*class) = 2;
												return;
											}
										}
									}
								}
							}
							else
							{
								if(attributes[57] <= 0.007577)
								{
									if(attributes[33] <= 0.007231)
									{
										if(attributes[86] <= 0.007614)
										{
											if(attributes[97] <= 0.007122)
											{
												// QR_4: 100%
												(*uncertainty) = 0.000000;
												(*class) = 4;
												return;
											}
											else
											{
												if(attributes[24] <= 0.006840)
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
												else
												{
													// QR_26: 100%
													(*uncertainty) = 0.000000;
													(*class) = 26;
													return;
												}
											}
										}
										else
										{
											if(attributes[43] <= 0.006623)
											{
												if(attributes[10] <= 0.005170)
												{
													// QR_1: 100%
													(*uncertainty) = 0.000000;
													(*class) = 1;
													return;
												}
												else
												{
													// QR_11: 100%
													(*uncertainty) = 0.000000;
													(*class) = 11;
													return;
												}
											}
											else
											{
												if(attributes[24] <= 0.006008)
												{
													if(attributes[33] <= 0.006199)
													{
														// QR_6: 100%
														(*uncertainty) = 0.000000;
														(*class) = 6;
														return;
													}
													else
													{
														// QR_2: 100%
														(*uncertainty) = 0.000000;
														(*class) = 2;
														return;
													}
												}
												else
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
											}
										}
									}
									else
									{
										if(attributes[44] <= 0.008730)
										{
											if(attributes[53] <= 0.008277)
											{
												if(attributes[83] <= 0.009213)
												{
													if(attributes[51] <= 0.006931)
													{
														// QR_2: 100%
														(*uncertainty) = 0.000000;
														(*class) = 2;
														return;
													}
													else
													{
														if(attributes[1] <= 0.108486)
														{
															// QR_1: 67%
															(*uncertainty) = 0.333333;
															(*class) = 1;
															return;
														}
														else
														{
															// QR_8: 100%
															(*uncertainty) = 0.000000;
															(*class) = 8;
															return;
														}
													}
												}
												else
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
											}
											else
											{
												if(attributes[64] <= 0.006886)
												{
													if(attributes[60] <= 0.006018)
													{
														// QR_8: 100%
														(*uncertainty) = 0.000000;
														(*class) = 8;
														return;
													}
													else
													{
														// QR_2: 100%
														(*uncertainty) = 0.000000;
														(*class) = 2;
														return;
													}
												}
												else
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
											}
										}
										else
										{
											if(attributes[62] <= 0.007453)
											{
												// QR_2: 99%
												(*uncertainty) = 0.017857;
												(*class) = 2;
												return;
											}
											else
											{
												if(attributes[68] <= 0.005726)
												{
													// QR_1: 100%
													(*uncertainty) = 0.000000;
													(*class) = 1;
													return;
												}
												else
												{
													// QR_13: 100%
													(*uncertainty) = 0.000000;
													(*class) = 13;
													return;
												}
											}
										}
									}
								}
								else
								{
									if(attributes[24] <= 0.007864)
									{
										if(attributes[94] <= 0.006702)
										{
											if(attributes[34] <= 0.007992)
											{
												if(attributes[48] <= 0.007405)
												{
													// QR_1: 100%
													(*uncertainty) = 0.000000;
													(*class) = 1;
													return;
												}
												else
												{
													// QR_19: 100%
													(*uncertainty) = 0.000000;
													(*class) = 19;
													return;
												}
											}
											else
											{
												// QR_2: 100%
												(*uncertainty) = 0.000000;
												(*class) = 2;
												return;
											}
										}
										else
										{
											if(attributes[55] <= 0.007400)
											{
												if(attributes[14] <= 0.006112)
												{
													// QR_5: 80%
													(*uncertainty) = 0.200000;
													(*class) = 5;
													return;
												}
												else
												{
													// QR_2: 100%
													(*uncertainty) = 0.000000;
													(*class) = 2;
													return;
												}
											}
											else
											{
												if(attributes[77] <= 0.007243)
												{
													// QR_1: 100%
													(*uncertainty) = 0.000000;
													(*class) = 1;
													return;
												}
												else
												{
													// QR_13: 100%
													(*uncertainty) = 0.000000;
													(*class) = 13;
													return;
												}
											}
										}
									}
									else
									{
										if(attributes[49] <= 0.006890)
										{
											if(attributes[7] <= 0.102507)
											{
												if(attributes[20] <= 0.003553)
												{
													// QR_1: 100%
													(*uncertainty) = 0.000000;
													(*class) = 1;
													return;
												}
												else
												{
													// QR_10: 100%
													(*uncertainty) = 0.000000;
													(*class) = 10;
													return;
												}
											}
											else
											{
												if(attributes[65] <= 0.006753)
												{
													if(attributes[0] <= 0.078651)
													{
														// QR_1: 100%
														(*uncertainty) = 0.000000;
														(*class) = 1;
														return;
													}
													else
													{
														// QR_11: 100%
														(*uncertainty) = 0.000000;
														(*class) = 11;
														return;
													}
												}
												else
												{
													// QR_4: 100%
													(*uncertainty) = 0.000000;
													(*class) = 4;
													return;
												}
											}
										}
										else
										{
											if(attributes[60] <= 0.007211)
											{
												if(attributes[23] <= 0.008214)
												{
													if(attributes[64] <= 0.008271)
													{
														// QR_4: 84%
														(*uncertainty) = 0.166667;
														(*class) = 4;
														return;
													}
													else
													{
														// QR_8: 100%
														(*uncertainty) = 0.000000;
														(*class) = 8;
														return;
													}
												}
												else
												{
													if(attributes[54] <= 0.008086)
													{
														// QR_7: 100%
														(*uncertainty) = 0.000000;
														(*class) = 7;
														return;
													}
													else
													{
														// QR_1: 100%
														(*uncertainty) = 0.000000;
														(*class) = 1;
														return;
													}
												}
											}
											else
											{
												if(attributes[51] <= 0.007091)
												{
													if(attributes[76] <= 0.008948)
													{
														if(attributes[12] <= 0.007328)
														{
															// QR_9: 100%
															(*uncertainty) = 0.000000;
															(*class) = 9;
															return;
														}
														else
														{
															// QR_2: 100%
															(*uncertainty) = 0.000000;
															(*class) = 2;
															return;
														}
													}
													else
													{
														// QR_1: 100%
														(*uncertainty) = 0.000000;
														(*class) = 1;
														return;
													}
												}
												else
												{
													// QR_6: 95%
													(*uncertainty) = 0.052632;
													(*class) = 6;
													return;
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			else
			{
				if(attributes[61] <= 0.008055)
				{
					if(attributes[8] <= 0.093738)
					{
						if(attributes[43] <= 0.008245)
						{
							if(attributes[12] <= 0.003545)
							{
								// QR_2: 100%
								(*uncertainty) = 0.000000;
								(*class) = 2;
								return;
							}
							else
							{
								// QR_20: 100%
								(*uncertainty) = 0.000000;
								(*class) = 20;
								return;
							}
						}
						else
						{
							// QR_1: 100%
							(*uncertainty) = 0.000000;
							(*class) = 1;
							return;
						}
					}
					else
					{
						if(attributes[57] <= 0.007384)
						{
							if(attributes[53] <= 0.009267)
							{
								if(attributes[20] <= 0.005927)
								{
									// QR_2: 100%
									(*uncertainty) = 0.000000;
									(*class) = 2;
									return;
								}
								else
								{
									// QR_23: 100%
									(*uncertainty) = 0.000000;
									(*class) = 23;
									return;
								}
							}
							else
							{
								// QR_2: 100%
								(*uncertainty) = 0.000000;
								(*class) = 2;
								return;
							}
						}
						else
						{
							if(attributes[63] <= 0.008227)
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
							else
							{
								// QR_10: 100%
								(*uncertainty) = 0.000000;
								(*class) = 10;
								return;
							}
						}
					}
				}
				else
				{
					if(attributes[64] <= 0.008977)
					{
						if(attributes[7] <= 0.109249)
						{
							// QR_2: 100%
							(*uncertainty) = 0.001401;
							(*class) = 2;
							return;
						}
						else
						{
							if(attributes[63] <= 0.006858)
							{
								// QR_2: 100%
								(*uncertainty) = 0.000000;
								(*class) = 2;
								return;
							}
							else
							{
								// QR_25: 100%
								(*uncertainty) = 0.000000;
								(*class) = 25;
								return;
							}
						}
					}
					else
					{
						if(attributes[22] <= 0.009959)
						{
							// QR_6: 100%
							(*uncertainty) = 0.000000;
							(*class) = 6;
							return;
						}
						else
						{
							// QR_1: 67%
							(*uncertainty) = 0.333333;
							(*class) = 1;
							return;
						}
					}
				}
			}
		}
		else
		{
			if(attributes[43] <= 0.006184)
			{
				if(attributes[94] <= 0.004649)
				{
					if(attributes[59] <= 0.008208)
					{
						// QR_2: 100%
						(*uncertainty) = 0.000000;
						(*class) = 2;
						return;
					}
					else
					{
						// QR_2: 75%
						(*uncertainty) = 0.250000;
						(*class) = 2;
						return;
					}
				}
				else
				{
					if(attributes[55] <= 0.008158)
					{
						if(attributes[54] <= 0.008182)
						{
							if(attributes[23] <= 0.009480)
							{
								// QR_1: 100%
								(*uncertainty) = 0.000000;
								(*class) = 1;
								return;
							}
							else
							{
								// QR_12: 100%
								(*uncertainty) = 0.000000;
								(*class) = 12;
								return;
							}
						}
						else
						{
							// QR_1: 99%
							(*uncertainty) = 0.010309;
							(*class) = 1;
							return;
						}
					}
					else
					{
						// QR_1: 100%
						(*uncertainty) = 0.000000;
						(*class) = 1;
						return;
					}
				}
			}
			else
			{
				if(attributes[65] <= 0.004503)
				{
					if(attributes[53] <= 0.007059)
					{
						// QR_9: 100%
						(*uncertainty) = 0.000000;
						(*class) = 9;
						return;
					}
					else
					{
						// QR_1: 100%
						(*uncertainty) = 0.000000;
						(*class) = 1;
						return;
					}
				}
				else
				{
					if(attributes[62] <= 0.007209)
					{
						if(attributes[53] <= 0.007836)
						{
							// QR_2: 100%
							(*uncertainty) = 0.001927;
							(*class) = 2;
							return;
						}
						else
						{
							if(attributes[51] <= 0.006905)
							{
								// QR_2: 100%
								(*uncertainty) = 0.000000;
								(*class) = 2;
								return;
							}
							else
							{
								// QR_26: 100%
								(*uncertainty) = 0.000000;
								(*class) = 26;
								return;
							}
						}
					}
					else
					{
						if(attributes[63] <= 0.006521)
						{
							// QR_1: 100%
							(*uncertainty) = 0.000000;
							(*class) = 1;
							return;
						}
						else
						{
							// QR_18: 100%
							(*uncertainty) = 0.000000;
							(*class) = 18;
							return;
						}
					}
				}
			}
		}
	}
}