# FROM ruby:2.7
FROM ruby:3.3

ENV LC_ALL C.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

WORKDIR /usr/src/app

RUN gem install bundler
# :2.4.22 
COPY Gemfile just-the-docs.gemspec ./
# RUN gem install bundler -v 2.4.22 && bundle install
RUN bundle install

EXPOSE 4000
