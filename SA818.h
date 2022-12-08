#ifndef SA818_H
#define SA818_H

#include <Stream.h>
#include <math.h>

class Request {
    public:
        char* req;
        int size;
        char** args;
};

class Response {
    public:
        Request* req;
        bool complete;
        String raw;
        String res;
};

enum Type { SA_818 = 0, SA_868 = 1 };

struct Conf {
    byte bw;
    int tx_f;
    int rx_f;
    int tx_sub;
    int sq;
    int rx_sub;
};

class SA818 {
    private:
        byte type_ = Type::SA_818;
        bool debug_ = false;
        Stream* stream_ = 0;
        long timeout_ = 1000;

        Response lastResponse_;

        Conf conf_ = {};
        // byte bw_ = 0;

        bool request(Request*);
        void print(char*);
        void flush();

        bool readByte(byte*);
        bool waitResponse(Response*);
        bool success(Response*);

        String ctcss(int);

        float loopScan(float, bool, float = 0);        

    public:
        SA818(Stream*);
        ~SA818();

        void set(Type);
        void debug(bool);
        void timeout(long = 1000);

        bool send(char*, int = 0, char** = 0);
        char* response();

        bool connect();
        bool setGroup(byte, float, float, int, byte, int);
        bool scan(float);
        bool setVolume(int);
        bool setFilter(int, int, int);
        bool openTail();
        bool closeTail();
        String rssi();
        String version();

        float next(float, float = 0);
        float previous(float, float = 0);
        
};

#endif

SA818::SA818(Stream* stream) : stream_(stream) {}

SA818::~SA818() {
    stream_ = 0;
    lastResponse_.req = 0;
}

void SA818::set(Type type) {
    type_ = type;
}

void SA818::debug(bool deb) {
    debug_ = deb;
}

bool SA818::send(char* cmd, int size, char** args) {
    lastResponse_.req = 0;

    Request req;
    req.req = cmd;
    req.size = size;
    req.args = args;

    bool ok = request(&req);
    return ok;
}

bool SA818::request(Request* req) {
    if(debug_)
        Serial.print("-> ");

    print(req->req);
    
    if(req->size > 0) 
        print("=");

    for(int i = 0; i < req->size; i++) {
        print(req->args[i]);
        if(i < req->size - 1) 
            print(",");
    }
    flush();

    Response resp;
    resp.req = req;
    resp.complete = false;

    waitResponse(&resp);

    if(debug_)
        Serial.println("<- " + resp.raw);

    bool ok = success(&resp);
    return ok;
}

void SA818::print(char* str) {
    if(debug_)
        Serial.print(str);
    stream_->print(str);
}

void SA818::flush() {
    if(debug_)
        Serial.println();
    stream_->print("\n\r\n");
}

bool SA818::readByte(byte* tk) {
    if(stream_->available() > 0) {
        *tk = stream_->read(); // Warning : the buffer is emptied
        return *tk != -1;
    }
    return false;
}

bool SA818::waitResponse(Response* resp) {
    String raw = "";
    String res = "";

    bool loop = true;
    long timer = millis();
    long time;
    byte zone = 0;
    do {
        byte tk = -1;
        if(readByte(&tk)) {
            if(tk != 13 && tk != 10) { // not [\r] or [\n]
                raw += (char) tk;

                if(zone == 1) {
                     res += (char) tk;
                }

                if(tk == 58 || tk == 61) { // [:] or [=]
                   zone++;
                }
            }

            if(tk == 10) { // [\n]
                loop = false;
                resp->complete = true;
            }
            else
                timer = millis();
        }

        time = millis() - timer;
        loop = time < timeout_;

    } while(loop);
    
    resp->raw = raw;
    resp->res = res;

    return resp->complete;
}

bool SA818::success(Response* resp) {
    if(resp->complete) {
        lastResponse_ = *resp;

        if(resp->res != "") {
            if(resp->res.equals("1")) return false;
            return true;
        }

        if(resp->raw.equals("+DMOERROR")) return false;
    }

    return false;
}

char* SA818::response() {
    return lastResponse_.raw.c_str();
}

String SA818::ctcss(int sub) {
    char s[4];
    if(sub < 0)
        sprintf(s, "%03dN", abs(sub));

    else if(sub >= 0 && sub < 23)
        sprintf(s, "%04d", sub);

    else if(sub >= 23) 
        sprintf(s, "%03dI", sub);

    return String(s);
}

bool SA818::connect() {
    return send("AT+DMOCONNECT");
}

bool SA818::setGroup(byte bw, float tx_f, float rx_f, int tx_sub, byte sq, int rx_sub) {
    conf_.bw = bw;
    conf_.tx_f = tx_f;
    conf_.rx_f = rx_f;
    conf_.tx_sub = tx_sub;
    conf_.sq = sq;
    conf_.rx_sub = rx_sub;

    String b = String(bw);
    String tf = String(tx_f, 4);
    String rf = String(rx_f, 4);
    String tsub = ctcss(tx_sub);
    String s = String(sq);
    String rsub = ctcss(rx_sub);
    char* params[] = {
        b.c_str(),
        tf.c_str(),
        rf.c_str(),
        tsub.c_str(),
        s.c_str(),
        rsub.c_str()
    };
    
    return send("AT+DMOSETGROUP", 6, params);
}

bool SA818::scan(float freq) {
    String cmd = String("S+") + String(freq, 4);
    return send(cmd.c_str());
}

bool SA818::setVolume(int vol) {
    String v =  String(vol);
    char* params[] = { v.c_str() };
    return send("AT+DMOSETVOLUME", 1, params);
}

bool SA818::setFilter(int emph, int high, int low) {
    String e = String(emph);
    String h = String(high);
    String l = String(low);
    char* params[] = { e.c_str(), h.c_str(), l.c_str() };
    return send("AT+SETFILTER", 3, params);
}

bool SA818::openTail() {
    return send("AT+SETTAIL=1");
}

bool SA818::closeTail() {
    return send("AT+SETTAIL=0");
}

String SA818::rssi() {
    String cmd = "RSSI?";
    if(type_ == SA_868)
        cmd = "AT+RSSI?";

    if(send(cmd.c_str()))
        return lastResponse_.res;
    return response();
}


String SA818::version() {
    if(send("AT+VERSION"))
        return lastResponse_.res;
    return response();
}

float SA818::loopScan(float mhz, bool dir, float khz) {
    // | GHz |     |     | MHz ||     |     | KHz ||     |     | Hz |
    // |     |     |     |     ||     |     |   1 ||   0 |   0 |  0 |
    // |     |     |     |   1 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |   1 |   0 |   0 |   0 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |     |   1 |   4 |   5 ||   7 |   5 |   0 ||   0 |   0 |  0 |
    // |     |     |     |     ||     |   1 |   2 ||   5 |   0 |  0 |
    
    float f = mhz * 1000; // MHz to KHz

    if(khz == 0) {
        khz = 12.5; // 12.5K

        if(conf_.bw == 1) // 25K
            khz = 25;
    }

    if(dir) f = f + khz;
    else f = f - khz;

    f = f / 1000;

    if(scan(f)) return f;
    else return loopScan(f, dir);
}

float SA818::next(float mhz, float khz) {
    return loopScan(mhz, true, khz);
}

float SA818::previous(float mhz, float khz) {
    return loopScan(mhz, false, khz);
}