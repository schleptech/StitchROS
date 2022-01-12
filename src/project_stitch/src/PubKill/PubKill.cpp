#include <stdlib.h>
#include <fstream>
#include <string.h>

void killProcess(std::string pName) {
    system(("pgrep " + pName + " > PID.txt").c_str());
    char command[15];
    memset(command, '\0', 15);

    strcat(command, "kill ");
    FILE* fp;
    fp = fopen("PID.txt", "r");
    fread(&command[5], 6, 1, fp);

    printf("%s",command);
    if(strlen(command) < 6) return;
    system(command);
}

int main(int argc, char* argv[]) {
    killProcess("RunRecorder");
    
    system("rosnode kill --all");
    return 0;
    
}