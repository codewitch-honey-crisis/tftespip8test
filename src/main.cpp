//#define ESP_WROVER_KIT
//#define PARALLEL8
#define LILYGO_TTGO
//#define ILI9341
#undef PARALLEL8
#ifdef PARALLEL8
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_BL -1
#define TFT_CS   33  // Chip select control pin (library pulls permanently low
#define TFT_DC   22  // (RS) Data Command control pin - must use a pin in the range 0-31
#define TFT_RST  32  // Reset pin, toggles on startup
#define TFT_WR    21  // Write strobe control pin - must use a pin in the range 0-31
#define TFT_RD    15  // Read strobe control pin
#define TFT_D0   2  // Must use pins in the range 0-31 for the data bus
#define TFT_D1   13  // so a single register write sets/clears all bits.
#define TFT_D2   26  // Pins can be randomly assigned, this does not affect
#define TFT_D3   25  // TFT screen update performance.
#define TFT_D4   27
#define TFT_D5   12
#define TFT_D6   14
#define TFT_D7   4
#elif defined(ESP_WROVER_KIT)
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_MISO 25
#define TFT_MOSI 23
#define TFT_SCLK 19
#define TFT_CS   22  // Chip select control pin (library pulls permanently low
#define TFT_DC   21  // (RS) Data Command control pin - must use a pin in the range 0-31
#define TFT_RST  18  // Reset pin, toggles on startup
#define TFT_BL   5            // LED back-light control pin
#define TFT_BACKLIGHT_ON LOW  // Level to turn ON back-light (HIGH or LOW)
#elif defined(LILYGO_TTGO)
#define TFT_WIDTH 135
#define TFT_HEIGHT 240
#define TFT_MISO -1
#define TFT_MOSI 19
#define TFT_SCLK  18
#define TFT_CS   5
#define TFT_DC   16
#define TFT_RST  23
#define TFT_BL 4
#else
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_MISO -1
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5  // Chip select control pin (library pulls permanently low
#define TFT_DC   2  // (RS) Data Command control pin - must use a pin in the range 0-31
#define TFT_RST  4  // Reset pin, toggles on startup
#define TFT_BL   15            // LED back-light control pin

#endif
#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
//#include <TFT_eSPI.h>
#include <gfx_cpp14.hpp>
//#include <gfx_tft_espi.hpp>
#include "tft_parallel8.hpp"
#include "tft_spi.hpp"
#if defined(PARALLEL8) || defined(ESP_WROVER_KIT) || defined(ILI9341)
#include "ili9341.hpp"
#else
#include "st7789.hpp"
#endif
#include "pretty_effect.hpp"
#include "../fonts/Bm437_ATI_9x16.h"
#include "../fonts/Bm437_Verite_9x16.h"
//#include "../fonts/Maziro.h"
using namespace gfx;
using namespace arduino;
#ifdef PARALLEL8
using bus_type = tft_p8<TFT_CS,TFT_WR,TFT_RD,TFT_D0,TFT_D1,TFT_D2,TFT_D3,TFT_D4,TFT_D5,TFT_D6,TFT_D7>;
#else
using bus_type = tft_spi<HSPI,TFT_CS,TFT_MOSI,TFT_MISO,TFT_SCLK,SPI_MODE0,
#ifdef LILYGO_TTGO
40*1000*1000
#elif defined(ESP_WROVER_KI)
40*1000*1000
#else
20*1000*1000
#endif
,20*1000*1000,true
#ifdef OPTIMIZE_DMA
,TFT_WIDTH*TFT_HEIGHT*2
#endif
>;
#endif
#if defined(PARALLEL8) || defined(ESP_WROVER_KIT) || defined(ILI9341)
using tft_type = ili9341<TFT_DC,TFT_RST,TFT_BL,bus_type,3
#ifdef ILI9341
,true
#endif
>;
#else
using tft_type = st7789<TFT_WIDTH,TFT_HEIGHT,TFT_DC,TFT_RST,TFT_BL,bus_type,3>;
#endif

using tft_color = color<typename tft_type::pixel_type>;

tft_type tft;

#define PARALLEL_LINES 16

using frame_buffer_type = large_bitmap<rgb_pixel<16>>;
using fb_color = color<typename frame_buffer_type::pixel_type>;


// prints a source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;+=x!1%$O@#";
    for(int y = 0;y<src.dimensions().height;++y) {
        for(int x = 0;x<src.dimensions().width;++x) {
            typename Source::pixel_type px;
            src.point(point16(x,y),&px);
            const auto px2 = convert<typename Source::pixel_type,gsc_pixel<4>>(px);
            size_t i =px2.template channel<0>();
            char sz[2] = {col_table[i],0};
            Serial.print(sz);
        }
        Serial.println();
    }
}

constexpr static const size16 bmp_size(16,16);
// you can use YbCbCr for example. It's lossy, so you'll want extra bits
//using bmp_type = bitmap<ycbcr_pixel<HTCW_MAX_WORD>>;
using bmp_type = bitmap<typename tft_type::pixel_type,palette<typename tft_type::pixel_type,typename tft_type::pixel_type>>;
using bmp_color = color<typename bmp_type::pixel_type>;
using bmpa_pixel_type = rgba_pixel<HTCW_MAX_WORD>;
using bmpa_color = color<bmpa_pixel_type>;
// declare the bitmap
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size,bmp_buf);

// produced by request
void scroll_text_demo() {
    tft.clear(tft.bounds());
    // draw stuff
    bmp.clear(bmp.bounds()); // comment this out and check out the uninitialized RAM. It looks neat.
    bmpa_pixel_type col = bmpa_color::yellow;
    col.channelr<channel_name::A>(.5);
    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1));
    rect16 ubounds(0,0,bounds.x2,bounds.y2);

    // draw the face
    draw::filled_ellipse(bmp,bounds,col);
    
    // draw the left eye
    srect16 eye_bounds_left(spoint16(bounds.width()/5,bounds.height()/5),ssize16(bounds.width()/5,bounds.height()/3));
    draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
    
    // draw the right eye
    srect16 eye_bounds_right(
        spoint16(
            bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
            eye_bounds_left.y1
        ),eye_bounds_left.dimensions());
    draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
    
    // draw the mouth
    srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,-bounds.height()/8).normalize();
    // we need to clip part of the circle we'll be drawing
    srect16 mouth_clip(mouth_bounds.x1,mouth_bounds.y1+mouth_bounds.height()/(float)1.6,mouth_bounds.x2,mouth_bounds.y2);
    draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);

    // do some alpha blended rectangles
    col = bmpa_color::red;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
    col = bmpa_color::blue;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
    col = bmpa_color::green;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,bmp.dimensions().height-bmp.dimensions().height/4),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
    col = bmpa_color::purple;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(bmp.dimensions().width-bmp.dimensions().width/4,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
    // uncomment to convert it to grayscale
    // resample<bmp_type,gsc_pixel<8>>(bmp);
    // uncomment to downsample
    // resample<bmp_type,rgb_pixel<8>>(bmp);
    srect16 new_bounds(0,0,63,63);

    // try using different values here. Bicubic yields the best visual result, but it's pretty slow. 
    // Bilinear is faster but better for shrinking images or changing sizes small amounts
    // Fast uses a nearest neighbor algorithm and is performant but looks choppy
    const bitmap_resize resize_type = 
         bitmap_resize::resize_bicubic;
        // bitmap_resize::resize_bilinear;
        //bitmap_resize::resize_fast;
    draw::bitmap(tft,new_bounds.center_horizontal((srect16)tft.bounds()).flip_vertical(),bmp,bmp.bounds(),resize_type);
    const gfx::font& f = Bm437_ATI_9x16_FON;
    const char* text = "copyright (C) 2021\r\nby honey the codewitch";
    ssize16 text_size = f.measure_text((ssize16)tft.dimensions(),text);
    srect16 text_rect = text_size.bounds().center((srect16)tft.bounds());
    int16_t text_start = text_rect.x1;
    
    // draw a polygon (a triangle in this case)
    // find the origin:
    const spoint16 porg = srect16(0,0,31,31)
                            .center_horizontal((srect16)tft.bounds())
                                .offset(0,
                                    tft.dimensions().height-32)
                                        .top_left();
    // draw a 32x32 triangle by creating a path
    spoint16 path_points[] = {spoint16(0,31),spoint16(15,0),spoint16(31,31)};
    spath16 path(3,path_points);
    // offset it so it starts at the origin
    path.offset_inplace(porg.x,porg.y); 
    // draw it
    draw::filled_polygon(tft,path,tft_color::coral);

    bool first=true;
    print_source(bmp);
    while(true) {

       draw::filled_rectangle(tft,text_rect,tft_color::black);
        if(text_rect.x2>=tft.dimensions().width) {
           draw::filled_rectangle(tft,text_rect.offset(-tft.dimensions().width,0),tft_color::black);
        }

        text_rect=text_rect.offset(2,0);
        draw::text(tft,text_rect,text,f,tft_color::old_lace,tft_color::black,false);
        if(text_rect.x2>=tft.dimensions().width){
            draw::text(tft,text_rect.offset(-tft.dimensions().width,0),text,f,tft_color::old_lace,tft_color::black,false);
        }
        if(text_rect.x1>=tft.dimensions().width) {
            text_rect=text_rect.offset(-tft.dimensions().width,0);
            first=false;
        }
        
        if(!first && text_rect.x1>=text_start)
            break;
    }
}
void lines_demo() {
    open_font f;
    // SHOULD EMBED THIS FOR MUCH FASTER RENDERING!!!:
    File file = SPIFFS.open("/Maziro.ttf");
    file_stream fs(file);
    open_font::open(&fs,&f);
    draw::filled_rectangle(tft,(srect16)tft.bounds(),tft_color::white);
    const char* text = "ESP32 GFX";
    float scale = f.scale(60);
    srect16 text_rect = f.measure_text((ssize16)tft.dimensions(),{0,0},
                            text,scale).bounds();
    draw::text(tft,
            text_rect.center((srect16)tft.bounds()),
            {0,0},
            text,
            f,scale,
            tft_color::dark_blue,tft_color::white,false);
    // free the font
    file.close();
    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(tft.dimensions().width/100.0),
                i*(tft.dimensions().height/100.0),
                tft.dimensions().width-i*(tft.dimensions().width/100.0)-1,
                tft.dimensions().height-i*(tft.dimensions().height/100.0)-1);
        // draw the four lines
        draw::line(tft,srect16(0,r.y1,r.x1,tft.dimensions().height-1),tft_color::light_blue);
        draw::line(tft,srect16(r.x2,0,tft.dimensions().width-1,r.y2),tft_color::hot_pink);
        draw::line(tft,srect16(0,r.y2,r.x1,0),tft_color::pale_green);
        draw::line(tft,srect16(tft.dimensions().width-1,r.y1,r.x2,tft.dimensions().height-1),tft_color::yellow);
        
    }
    
}

//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors()
{
    using lines_bmp_type = bitmap<typename tft_type::pixel_type>;
    lines_bmp_type line_bmps[2] {
        lines_bmp_type(size16(320,PARALLEL_LINES),malloc(320*PARALLEL_LINES*sizeof(uint16_t))),
        lines_bmp_type(size16(320,PARALLEL_LINES),malloc(320*PARALLEL_LINES*sizeof(uint16_t)))
    };
    
    int frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
 
#ifdef ASCII_JPEGS
    bool print=true;
#endif
    while(true) {
        if(0==frame%150) {
            lines_demo();
            scroll_text_demo();
        }
        ++frame;
        if(0!=(frame%10))
            vTaskDelay(1);
        for (int y=0; y<240; y+=PARALLEL_LINES) {
            //Calculate a line.
            pretty_effect_calc_lines(320,240, line_bmps[calc_line], y, frame, PARALLEL_LINES);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            const lines_bmp_type& sending_bmp = line_bmps[sending_line];
            rect16 src_bounds = sending_bmp.bounds();
#ifdef ASCII_JPEGS
            if(print) {
                if(y+PARALLEL_LINES>=240)
                    print=false;
                print_source(sending_bmp);
            }
#endif
            draw::wait_all_async(tft);
            draw::bitmap_async(tft,(srect16)src_bounds.offset(0,y),sending_bmp,src_bounds);
            
        }
        if(0==frame%50) {
            using tft_color = color<rgb_pixel<16>>;
            int pid = (frame/50)%3;
            
            if(pid==1) {
                for(int i=0;i<60;++i) {
                    srect16 sr(spoint16(rand()%tft.dimensions().width,rand()%tft.dimensions().height),rand()%(tft.dimensions().width/4));
                    draw::filled_ellipse(tft, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                }
            } else if(pid==2) {
                for(int i=0;i<90;++i) {
                    srect16 sr(spoint16(rand()%tft.dimensions().width,rand()%tft.dimensions().height),rand()%(tft.dimensions().width/4));
                    if(0!=(rand()%2)) {
                        draw::filled_rectangle(tft, sr,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    } else {
                        draw::filled_rounded_rectangle(tft, sr,(rand()%10)/10.0,rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    }
                }
            } else {
                for(int i = 1;i<120;++i) {
                    draw::line(tft,srect16(0,i*(tft.dimensions().height/240.0),tft.dimensions().width-1,i*(tft.dimensions().height/240.0)),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(i*(tft.dimensions().width/240.0),0,i*(tft.dimensions().width/240.0),tft.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,0,tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,tft.dimensions().height-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                    draw::line(tft,srect16(0,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1,tft.dimensions().width-1,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1),rgb_pixel<16>(rand()%32,rand()%64,rand()%32));
                }
                for(int i = 1;i<120;++i) {
                    draw::line(tft,srect16(0,i*(tft.dimensions().height/240.0),tft.dimensions().width-1,i*(tft.dimensions().height/240.0)),tft_color::black);
                    draw::line(tft,srect16(i*(tft.dimensions().width/240.0),0,i*(tft.dimensions().width/240.0),tft.dimensions().height-1),tft_color::black);
                    draw::line(tft,srect16(tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,0,tft.dimensions().width-i*(tft.dimensions().width/240.0)-1,tft.dimensions().height-1),tft_color::black);
                    draw::line(tft,srect16(0,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1,tft.dimensions().width-1,tft.dimensions().height-i*(tft.dimensions().height/240.0)-1),tft_color::black);
                }
            }
#ifdef LILYGO_TTGO
            File fs = SPIFFS.open("/image_240.jpg");
#else
            File fs = SPIFFS.open((0==pid)?"/image.jpg":(1==pid)?"/image2.jpg":"/image3.jpg");
#endif
            draw::image(pixels,(srect16)pixels.bounds(),&fs,rect16(0,0,-1,-1));
            fs.close();
#ifdef ASCII_JPEGS
            print=true;
#endif
        }
    }
}

void setup() {
    Serial.begin(115200);
    SPIFFS.begin(false);
    //tft_espi.init();
    //tft_espi.setRotation(1);
#ifdef LILYGO_TTGO
    pretty_effect_init("/image_240.jpg",256,151,320,240);
#else
    pretty_effect_init("/image.jpg",336,256,tft.dimensions().width,tft.dimensions().height);
#endif
    
    display_pretty_colors();
    //tft.fill(tft.bounds(),tft_color::goldenrod);
    //typename tft_type::pixel_type px;
    //tft.point({1,2},&px);
    //Serial.printf("(%d, %d, %d)\r\n",(int)px.channel<0>(),(int)px.channel<1>(),(int)px.channel<2>());
}
void loop() {

}

