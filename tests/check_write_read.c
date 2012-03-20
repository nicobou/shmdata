int
check_read_write ()
{
  return 0;
}

int
main (int argc, char * argv)
{
  if (check_read_write() != 0)
    return 1;
  return 0;
}

