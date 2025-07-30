module line_buffer (
    input wire clk,
    input wire hsync,
    input wire vsync,
    input wire hblank,
    input wire vblank,
    input wire [11:0] pixel,

    output wire hsync_out,
    output wire vsync_out,
    output wire hblank_out,
    output wire vblank_out,
    output wire [11:0] pixel_out
);

reg [11:0] ping_pixels [511:0];
reg ping_hsync [511:0];
reg ping_vsync [511:0];
reg ping_hblank [511:0];
reg ping_vblank [511:0];

reg [11:0] pong_pixels [511:0];
reg pong_hsync [511:0];
reg pong_vsync [511:0];
reg pong_hblank [511:0];
reg pong_vblank [511:0];

reg ping_pong;

reg [11:0] index;

reg last_hsync;

always @(posedge clk) begin
    if (ping_pong == 1'b1) begin
        ping_hsync[index] = hsync;
        ping_vsync[index] = vsync;
        ping_vblank[index] = vblank;
        ping_hblank[index] = hblank;
        ping_pixels[index] = pixel;

        hsync_out = pong_hsync[index];
        vsync_out = pong_vsync[index];
        vblank_out = pong_vblank[index];
        hblank_out = pong_hblank[index];
        if (index < 12'd8) begin
            pixel_out = 12'd0;
        end
        else begin
            pixel_out = pong_pixels[index + 12'd9];
        end
    end
    else begin
        pong_hsync[index] = hsync;
        pong_vsync[index] = vsync;
        pong_vblank[index] = vblank;
        pong_hblank[index] = hblank;
        pong_pixels[index] = pixel;

        hsync_out = ping_hsync[index];
        vsync_out = ping_vsync[index];
        vblank_out = ping_vblank[index];
        hblank_out = ping_hblank[index];
        if (index < 12'd8) begin
            pixel_out = 12'd0;
        end
        else begin
            pixel_out = ping_pixels[index + 12'd9];
        end
    end

    index = index + 12'd1;
    if (hsync && !last_hsync) begin
        index = 0;
        ping_pong = ~ping_pong;
    end
    last_hsync = hsync;
end

endmodule
