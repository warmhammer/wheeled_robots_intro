# Принцип сдачи ДЗ
При чтении этой инструкции стоит внимательно читать и её саму, и вывод git. Если что-то пошло не так, то всегда можно спросить в чате.

## Подготовка

1) Для того, чтобы иметь возможность сохранять изменения кода на вашем компьютере на GitHub, нужно
"сообщить" GitHub`у каким образом идентифицировать ваш компьютер. Для этого существует поняние ssh-ключей. Для генерации пары открытый/закрытый ssh-ключ исполните следующие команды (либо посмотрите этот [туториал](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux)):
```
$ mkdir -p ~/.ssh
$ chmod 700 ~/.ssh
$ ssh-keygen -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/Users/p.bereznoy/.ssh/id_rsa):
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in /tmp/test.
Your public key has been saved in /tmp/test.pub.
The key fingerprint is:
SHA256:9Yf53peICFiHtsvlLTU+lnMidP9Vd7U7aWtSE9GWyIs p.bereznoy@msk-wifi-21fap7-p_berezhnoy-noname.mail.msk
The keys randomart image is:
+---[RSA 2048]----+
|            . . o|
|             o oo|
|        . . . ..o|
|       + o E + .o|
|      + S   + ..=|
|     . o o + o o*|
|      . * * = +=+|
|       o = X *.+=|
|          + = =oo|
+----[SHA256]-----+
```

Если вам не нужен пароль для ключа (большинству не нужно), тоже просто нажмите Enter.

В результате у вас были сгенерированы 2 файла:
```
$ ls ~/.ssh
id_rsa id_rsa.pub
```

2) Необходимо скопировать содержимое файла `id_rsa.pub` на GitHub:
```
$ cat ~/.ssh/id_rsa.pub
ssh-rsa ...
```
[Добавляем ключ](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux) в аккаунт.

3) Создаем приватный "форк" (можно следовать [туториалу](https://docs.github.com/ru/repositories/creating-and-managing-repositories/duplicating-a-repository)). 

Прежде всего в своем аккаунте на GitHub создает пустой приватный репозиторий (лучше использовать название `wheeled_robots_intro` во избежание путаницы). Добавляем пользователя `warmhammer` (Settings -> Collaborators).

Открываем bash, переходим в рабочую папку и создаем "чистый" клон репозитория.
```
$ git clone --bare https://github.com/warmhammer/wheeled_robots_intro.git
```

Отправляем зеркало в новый репозиторий:
```
$ cd wheeled_robots_intro.git
$ git push --mirror https://github.com/<USER>/wheeled_robots_intro.git
```

Удаляем временный локальный репозиторий, созданный ранее:
```
$ cd ..
$ rm -rf wheeled_robots_intro.git
```

4) [Клонируем репозиторий](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo#cloning-your-forked-repository) на локальную машину:

```
$ git clone git@github.com:<USER>/wheeled_robots_intro.git
```

## Выполнение ДЗ

Перед тем как начать делать задание, вам необходимо создать новую ветку с именем `hw-num`, где `num` - номер задания, от ветки `main`:

```
# Создаем ветку и переходим в неё
$ git checkout -b hw-1
```

После этого нужно делать домашнее задание в этой ветке. Используйте выделенную директорию под каждое дз!

## Сдача ДЗ

После выполнения дз нужно залить все изменения в удаленный репозиторий. Дальше [делается pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/merging-a-pull-request) из `hw-num` ветки в `main` своего форка (не базового репозитория!). Ссылку на PR сдается в форму.

**Делать merge самостоятельно не надо! Это делает преподаватель после того, как даст комментарии к работе.**

## Получение следующих дз

Для того, чтобы новые дз появлялись в вашем форке, воспользуйтесь ручным зеркалированием.

Добавьте преподавательский репозиторий в список удаленных репозиториев (делается один раз):

```
$ git remote add upstream "git@github.com:warmhammer/wheeled_robots_intro.git"
```

Изменяем политику merge:
```
$ git config pull.rebase false
```

Для добавления всех изменений в локальный репозиторий переходим в ветку `main`:
```
$ git checkout main
```
Стягиваем изменения и мержим удаленную ветку `main` в локальную:
```
$ git pull upstream main
```