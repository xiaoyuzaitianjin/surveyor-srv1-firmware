main()
{
    int x;
    char ch;
 
    ch = 0;
    while (ch == 0) {
        x = range();   /* user laser pointer ranging */
        print("range = " x);
        if (x < 30) {
            motors(-50 , 50);
        } else {
            motors(50, 50);
        }
        delay(500);
        ch = input();  /* continue until any console input */
    } 
    motors(0, 0);
}

